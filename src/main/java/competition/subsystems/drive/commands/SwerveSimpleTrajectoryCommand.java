package competition.subsystems.drive.commands;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import xbot.common.command.BaseCommand;
import xbot.common.controls.sensors.XTimer;
import xbot.common.math.XYPair;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.drive.control_logic.HeadingModule;
import xbot.common.subsystems.drive.control_logic.HeadingModule.HeadingModuleFactory;

import javax.inject.Inject;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class SwerveSimpleTrajectoryCommand extends BaseCommand {

    DriveSubsystem drive;
    PoseSubsystem pose;
    HeadingModule headingModule;

    private Supplier<List<XbotSwervePoint>> keyPointsProvider;

    private List<XbotSwervePoint> keyPoints;

    private XbotSwervePoint baselinePoint;

    double maxPower = 1.0;
    double maxTurningPower = 1.0;
    double accumulatedProductiveSeconds = 0;
    double previousTimestamp = 0;
    double maximumDistanceFromChasePointInInches = 24;
    int targetIndex = 0;
    double lerpFraction;

    private final Field2d ghostDisplay;

    @Inject
    public SwerveSimpleTrajectoryCommand(DriveSubsystem drive, PoseSubsystem pose, PropertyFactory pf, HeadingModuleFactory headingModuleFactory) {
        this.drive = drive;
        this.pose = pose;
        headingModule = headingModuleFactory.create(drive.getRotateToHeadingPid());

        ghostDisplay = new Field2d();
        SmartDashboard.putData("Ghost", ghostDisplay);

        pf.setPrefix(this);
        this.addRequirements(drive);
    }

    // --------------------------------------------------------------
    // Configuration
    // --------------------------------------------------------------

    public void setKeyPoints(List<XbotSwervePoint> keyPoints) {
        setKeyPointsProvider(() -> keyPoints);
    }

    public void setKeyPointsProvider(Supplier<List<XbotSwervePoint>> keyPointsProvider) {
        this.keyPointsProvider = keyPointsProvider;
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }

    public void setMaxTurningPower(double maxTurningPower) {
        this.maxTurningPower = maxTurningPower;
    }

    // --------------------------------------------------------------
    // Major Command Elements
    // --------------------------------------------------------------

    @Override
    public void initialize() {
        log.info("Initializing");
        keyPoints = keyPointsProvider.get();
        log.info("Key points size: " + keyPoints.size());
        baselinePoint = new XbotSwervePoint(pose.getCurrentPose2d(), 0);
        previousTimestamp = XTimer.getFPGATimestamp();
        targetIndex = 0;
        accumulatedProductiveSeconds = 0;
    }

    @Override
    public void execute() {

        double secondsSinceLastExecute = XTimer.getFPGATimestamp() - previousTimestamp;
        accumulatedProductiveSeconds += secondsSinceLastExecute;

        // If we somehow have no points to visit, don't do anything.
        if (keyPoints.size() == 0) {
            drive.stop();
            return;
        }

        // Linearly interpolate between the baselinePoint and the first keyPoint.
        var targetKeyPoint = keyPoints.get(targetIndex);

        if (targetKeyPoint.secondsToPoint <= 0) {
            log.error("Cannot have a keypoint with a time of 0 or less!");
            drive.stop();
            return;
        }

        // First, assume we are just going to our target. (This is what all trajectories will eventually
        // settle to - all this interpolation is for intermediate points.)
        Translation2d chasePoint = targetKeyPoint.keyPose.getTranslation();

        // Now, try to find a better point via linear interpolation.
        lerpFraction = (accumulatedProductiveSeconds) / targetKeyPoint.secondsToPoint;

        // If the fraction is above 1, it's time to set a new baseline point and start LERPing on the next
        // one.
        if (lerpFraction >= 1 && targetIndex < keyPoints.size()-1) {
            // What was our target now becomes our baseline.
            baselinePoint = targetKeyPoint;
            accumulatedProductiveSeconds = 0;

            targetIndex++;
            // And set our new target to the next element of the list
            targetKeyPoint = keyPoints.get(targetIndex);
        }

        // Most of the time, the fraction will be less than one.
        // In that case, we want to interpolate between the baseline and the target.
        if (lerpFraction < 1) {
            chasePoint = baselinePoint.keyPose.getTranslation().interpolate(
                    targetKeyPoint.keyPose.getTranslation(), lerpFraction);
        }

        // But if that chase point is "too far ahead", we need to freeze the chasePoint
        // until the robot has a chance to catch up.
        if (pose.getCurrentPose2d().getTranslation().getDistance(chasePoint) > maximumDistanceFromChasePointInInches) {
            // This effectively "rewinds time" for the next loop.
            accumulatedProductiveSeconds -= secondsSinceLastExecute;
        }

        // Update the ghost display.
        ghostDisplay.setRobotPose(new Pose2d(chasePoint.div(PoseSubsystem.INCHES_IN_A_METER), targetKeyPoint.keyPose.getRotation()));

        // Now that we have a chase point, we can drive to it. The rest of the logic is
        // from our proven SwerveToPointCommand. Eventually, the common components should be
        // refactored and should also move towards WPI objects (e.g. Pose2d rather than FieldPose).

        XYPair targetPosition = new XYPair(chasePoint.getX(), chasePoint.getY());

        // Get the difference between where we are, and where we want to be.
        XYPair goalVector = targetPosition.clone().add(
                pose.getCurrentFieldPose().getPoint().scale(-1)
        );

        // PID on the magnitude of the goal. Kind of similar to rotation,
        // our goal is "zero error".
        double magnitudeGoal = goalVector.getMagnitude();
        double drivePower = drive.getPositionalPid().calculate(magnitudeGoal, 0);

        // Create a vector in the direction of the goal, scaled by the drivePower.
        XYPair intent = XYPair.fromPolar(goalVector.getAngle(), drivePower);

        double headingPower = headingModule.calculateHeadingPower(
                targetKeyPoint.keyPose.getRotation().getDegrees());

        if (intent.getMagnitude() > maxPower && maxPower > 0 && intent.getMagnitude() > 0) {
            intent = intent.scale(maxPower / intent.getMagnitude());
        }

        if (maxTurningPower > 0)
        {
            headingPower = headingPower * maxTurningPower;
        }

        drive.fieldOrientedDrive(intent, headingPower, pose.getCurrentHeading().getDegrees(), false);
        previousTimestamp = XTimer.getFPGATimestamp();

        Pose2d pose = new Pose2d();
    }

    @Override
    public boolean isFinished() {
        boolean finished = drive.getPositionalPid().isOnTarget() && headingModule.isOnTarget()
                && targetIndex == keyPoints.size()-1 & lerpFraction >= 1;
        if (finished) {
            log.info("Finished");
            log.info("TargetIndex" + targetIndex + ", KeyPoints Size" + keyPoints.size());
            log.info("LerpFraction" + lerpFraction);
        }
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        drive.stop();
    }
}