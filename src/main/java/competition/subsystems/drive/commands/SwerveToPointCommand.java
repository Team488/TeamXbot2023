package competition.subsystems.drive.commands;

import java.util.function.Supplier;

import javax.inject.Inject;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import xbot.common.command.BaseCommand;
import xbot.common.math.XYPair;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.drive.control_logic.HeadingModule;
import xbot.common.subsystems.drive.control_logic.HeadingModule.HeadingModuleFactory;

public class SwerveToPointCommand extends BaseCommand {

    DriveSubsystem drive;
    PoseSubsystem pose;
    DoubleProperty directionToTarget;
    HeadingModule headingModule;

    private Supplier<XYPair> targetPositionSupplier;
    private Supplier<Double> targetHeadingSupplier;

    private XYPair targetPosition;
    private double targetHeading;

    private boolean robotRelativeMotion = false;

    double maxPower = 1.0;
    double maxTurningPower = 1.0;

    @Inject
    public SwerveToPointCommand(DriveSubsystem drive, PoseSubsystem pose, PropertyFactory pf, HeadingModuleFactory headingModuleFactory) {
        this.drive = drive;
        this.pose = pose;
        headingModule = headingModuleFactory.create(drive.getRotateToHeadingPid());

        pf.setPrefix(this);
        directionToTarget = pf.createEphemeralProperty("Direction to target", 0);
        this.addRequirements(drive);
    }

    @Override
    public void initialize() {
        log.info("Initializing");

        targetHeading = targetHeadingSupplier.get();
        targetPosition = targetPositionSupplier.get().clone();

        if (robotRelativeMotion) {
            // If we are using robot relative motion, we need to consider the target position
            // as being relative to the robot's current position. So value of -60,0 means
            // "go backwards 60 inches" from the robot's perspective.
            // If the robot was pointed at field -90 degrees (right) this would mean setting
            // a field-relative target of 0,60 (move the robot left 60 inches).
            targetPosition.rotate(pose.getCurrentHeading().getDegrees());
            
            targetPosition = pose.getCurrentFieldPose().getPoint().clone().add(targetPosition);

            // Then, move the target heading itself. 
            targetHeading = pose.getCurrentHeading().getDegrees() + targetHeading;
        }

        log.info(String.format("Swerve to point targets: (%f, %f), %f", targetPosition.x, targetPosition.y, targetHeading));
    }

    public XYPair getActiveTargetPosition() {
        return targetPosition;
    }

    public double getActiveHeading() {
        return targetHeading;
    }

    public void setTargetPosition(XYPair targetPositionInInches, double heading) {
        this.targetPositionSupplier = () -> targetPositionInInches;
        this.targetHeadingSupplier = () -> heading;
    }

    public void setTargetSupplier(Supplier<XYPair> targetPositionSupplier, Supplier<Double> targetHeadingSupplier) {
        this.targetPositionSupplier = targetPositionSupplier;
        this.targetHeadingSupplier = targetHeadingSupplier;
    }

    public void setRobotRelativeMotion() {
        robotRelativeMotion = true;
    }

    public void setFieldRelativeMotion() {
        robotRelativeMotion = false;
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }

    public void setMaxTurningPower(double maxTurningPower) {
        this.maxTurningPower = maxTurningPower;
    }
    @Override
    public void execute() {
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

        double headingPower = headingModule.calculateHeadingPower(targetHeading);

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
        return drive.getPositionalPid().isOnTarget() && headingModule.isOnTarget();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        drive.stop();
    }
}