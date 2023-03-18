package competition.subsystems.drive.commands;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import competition.trajectory.SimpleTimeInterpolator;
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

    double maxPower = 1.0;
    double maxTurningPower = 1.0;

    private final SimpleTimeInterpolator interpolator = new SimpleTimeInterpolator();
    SimpleTimeInterpolator.InterpolationResult lastResult;

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
        interpolator.setKeyPoints(keyPoints);
        interpolator.initialize(new XbotSwervePoint(pose.getCurrentPose2d(), 0));
    }

    @Override
    public void execute() {

        var currentPosition = pose.getCurrentPose2d();
        lastResult = interpolator.calculateTarget(currentPosition.getTranslation());
        var chasePoint = lastResult.chasePoint;

        // Update the ghost display.
        ghostDisplay.setRobotPose(new Pose2d(chasePoint.div(PoseSubsystem.INCHES_IN_A_METER), lastResult.chaseHeading));

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
                lastResult.chaseHeading.getDegrees());

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
        boolean finished = drive.getPositionalPid().isOnTarget() && headingModule.isOnTarget()
                && lastResult.isOnFinalPoint;
        if (finished) {
            log.info("Finished");
        }
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        drive.stop();
    }
}