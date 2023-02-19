package competition.subsystems.drive.commands;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import xbot.common.command.BaseCommand;
import xbot.common.controls.sensors.XTimer;
import xbot.common.math.XYPair;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.drive.control_logic.HeadingModule;
import xbot.common.subsystems.drive.control_logic.HeadingModule.HeadingModuleFactory;

import javax.inject.Inject;
import java.util.List;
import java.util.function.Supplier;

public class SwerveSimpleTrajectoryCommand extends BaseCommand {

    DriveSubsystem drive;
    PoseSubsystem pose;
    DoubleProperty directionToTarget;
    HeadingModule headingModule;

    private Supplier<List<XbotSwervePoint>> keyPointsProvider;

    private List<XbotSwervePoint> keyPoints;

    private XbotSwervePoint baselinePoint;

    double maxPower = 1.0;
    double maxTurningPower = 1.0;
    double lerpTime = 0;

    @Inject
    public SwerveSimpleTrajectoryCommand(DriveSubsystem drive, PoseSubsystem pose, PropertyFactory pf, HeadingModuleFactory headingModuleFactory) {
        this.drive = drive;
        this.pose = pose;
        headingModule = headingModuleFactory.create(drive.getRotateToHeadingPid());

        pf.setPrefix(this);
        this.addRequirements(drive);
    }

    // --------------------------------------------------------------
    // Configuration
    // --------------------------------------------------------------

    public void setKeyPoints(List<XbotSwervePoint> keyPoints) {
        this.keyPoints = keyPoints;
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

        baselinePoint = new XbotSwervePoint(pose.getCurrentPose2d(), 0);
        lerpTime = XTimer.getFPGATimestamp();
    }

    @Override
    public void execute() {

        // If we somehow have no points to visit, don't do anything.
        if (keyPoints.size() == 0) {
            drive.stop();
            return;
        }

        // Linearly interpolate between the baselinePoint and the first keyPoint.
        var targetKeyPoint = keyPoints.get(0);

        if (targetKeyPoint.secondsToPoint <= 0) {
            log.error("Cannot have a keypoint with a time of 0 or less!");
            drive.stop();
            return;
        }

        // First, assume we are just going to our target. (This is what all trajectories will eventually
        // settle to - all this interpolation is for intermediate points.)
        Translation2d chasePoint = targetKeyPoint.keyPose.getTranslation();

        // Now, try to find a better point via linear interpolation.
        double lerpFraction = (XTimer.getFPGATimestamp() - lerpTime) / targetKeyPoint.secondsToPoint;

        // If the fraction is above 1, it's time to set a new baseline point and start LERPing on the next
        // one.
        if (lerpFraction >= 1 && keyPoints.size() > 1) {
            // What was our target now becomes our baseline.
            baselinePoint = targetKeyPoint;
            lerpTime = XTimer.getFPGATimestamp();

            // Remove our target from the list of points to visit
            keyPoints.remove(0);
            // And set our new target to what is now the first point in the list.
            targetKeyPoint = keyPoints.get(0);
        }

        // Most of the time, the fraction will be less than one.
        // In that case, we want to interpolate between the baseline and the target.
        if (lerpFraction < 1) {
            chasePoint = baselinePoint.keyPose.getTranslation().interpolate(
                    targetKeyPoint.keyPose.getTranslation(), lerpFraction);
        }

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
        directionToTarget.set(goalVector.getAngle());

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
    }

    @Override
    public boolean isFinished() {
        return drive.getPositionalPid().isOnTarget() && headingModule.isOnTarget() && keyPoints.size() == 1;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        drive.stop();
    }
}