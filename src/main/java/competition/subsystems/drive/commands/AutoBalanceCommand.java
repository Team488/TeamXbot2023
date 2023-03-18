package competition.subsystems.drive.commands;

import javax.inject.Inject;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import org.littletonrobotics.junction.Logger;
import xbot.common.command.BaseCommand;
import xbot.common.controls.sensors.XTimer;
import xbot.common.math.PIDManager;
import xbot.common.math.PIDManager.PIDManagerFactory;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;
import xbot.common.properties.StringProperty;

public class AutoBalanceCommand extends BaseCommand {
    
    private final DriveSubsystem drive;
    private final PoseSubsystem pose;
    private final PIDManager pidManager;

    private boolean drivingAgainstPositiveAngle = true;
    final DoubleProperty firstAttemptSpeedProperty;
    final DoubleProperty speedMultiplierProperty;
    double currentAttemptSpeed;
    double lastDetectedFallTime = -1000;
    double initialAnalyzedAngle = 0;
    double angleThresholdForBalance = 1.5;

    public enum BalanceState {
        Analyzing,
        Driving,
        FallDetected,
        Waiting,
        Complete
    }

    private BalanceState currentBalanceState = BalanceState.Analyzing;

    @Inject
    public AutoBalanceCommand(DriveSubsystem drive, PoseSubsystem pose, PIDManagerFactory pidFactory,
                              PropertyFactory pf) {
        this.drive = drive;
        this.pose = pose;
        pf.setPrefix(this);
        this.pidManager = pidFactory.create(
            this.getPrefix(),
            0.01,// P
            0, // I
            0, // D
            0.25, // Max Output
            -0.25); // Min Output
            
        this.firstAttemptSpeedProperty = pf.createPersistentProperty("firstAttemptSpeed", 0.35);
        speedMultiplierProperty = pf.createPersistentProperty("speedMultiplier", 0.5);
    }

    // Basic idea:
    // - Assume that we start at either 15 or -15 degrees
    // - Drive the correct direction at constant velocity
    // - Once we notice a significant change in angle (15->12 or -15->-12), lock the wheels for a bit
    // - After a short time (~1 second?) check the angle and drive in the new correct direction at constant,
    //   but slower, velocity.
    //   Repeat until our resting angle after wheel lock is +/- 1.5 degrees.

    @Override
    public void initialize() {
        log.info("Initializing");
        pidManager.reset();

        drivingAgainstPositiveAngle = getAngle() > 0.0;
        currentAttemptSpeed = firstAttemptSpeedProperty.get();
        currentBalanceState = BalanceState.Analyzing;
    }

    @Override
    public void execute() {
        // Let's assume that we are already strongly "on the ramp", which is to say, at or very close
        // to the resting angle of the robot, which is +/-15 degrees.

        double currentAngle = getAngle();

        double velocityGoal = 0;
        switch (currentBalanceState) {
            case Analyzing:
                log.info("Analyzing");
                drivingAgainstPositiveAngle = currentAngle > 0.0;

                if (isRobotLevel()) {
                    // We're already balanced, so we're done!
                    currentBalanceState = BalanceState.Complete;
                    break;
                }

                currentBalanceState = BalanceState.Driving;
                drive.setActivateBrakeOverride(false);
                initialAnalyzedAngle = currentAngle;
                break;
            case Driving:

                // If we are driving against the positive angle, we need to use positive velocity to
                // climb the ramp. If we are driving against negative angle, we need negative velocity
                // to climb the ramp.

                if (drivingAgainstPositiveAngle) {
                    velocityGoal = currentAttemptSpeed;
                } else {
                    velocityGoal = -currentAttemptSpeed;
                }

                // If we're driving against a positive angle and see an even more positive angle, we need
                // to update our angle to that angle (and the mirror for negative angle).
                // This could happen if we only are kind of on the leaf of the ramp and then our angle
                // increases as we leave the leaf and get on the main body of the charge pad.
                if ((drivingAgainstPositiveAngle && currentAngle > initialAnalyzedAngle)
                 || (!drivingAgainstPositiveAngle && currentAngle < initialAnalyzedAngle)) {
                    initialAnalyzedAngle = currentAngle;
                }

                // If we're driving against the positive angle, see a sudden drop in angle, we need
                // to advance to FallDetected. Likewise, if we're driving against the negative angle,
                // and we see a sudden increase in angle, we need to advance to FallDetected.
                double fallThreshold = 2;
                if (drivingAgainstPositiveAngle && initialAnalyzedAngle-currentAngle > fallThreshold) {
                    currentBalanceState = BalanceState.FallDetected;
                } else if (!drivingAgainstPositiveAngle && initialAnalyzedAngle-currentAngle < -fallThreshold) {
                    currentBalanceState = BalanceState.FallDetected;
                }

                break;
            case FallDetected:
                log.info("Fall detected");
                // We've detected a fall, so we need to stop driving and wait for a bit.
                // We'll also slow down the drive speed for the next attempt.
                currentAttemptSpeed = currentAttemptSpeed * speedMultiplierProperty.get();
                currentBalanceState = BalanceState.Waiting;
                lastDetectedFallTime = XTimer.getFPGATimestamp();
                velocityGoal = 0;
                drive.setActivateBrakeOverride(true);
                break;
            case Waiting:
                if (XTimer.getFPGATimestamp() - lastDetectedFallTime > 2.0) {
                    if (isRobotLevel()) {
                        // We've successfully balanced!
                        currentBalanceState = BalanceState.Complete;
                    } else {
                        // We're still not balanced, so we need to try again.
                        currentBalanceState = BalanceState.Analyzing;
                    }
                }
                velocityGoal = 0;
                break;
            case Complete:
                log.info("Complete");
                velocityGoal = 0;
                return;
            default:
                // How did this even happen?
                break;
        }

        Logger.getInstance().recordOutput(getPrefix()+"BalanceState", currentBalanceState.toString());
        drive.setVelocityMaintainerXTarget(velocityGoal);
    }

    private double getAngle() {
        // this is based on the rio orientation
        double currentAngle = -(pose.getRobotPitch());

        if (Math.abs(currentAngle) < angleThresholdForBalance) {
            currentAngle = 0;
        }

        return currentAngle;
    }

    private boolean isRobotLevel() {
        return Math.abs(getAngle()) < angleThresholdForBalance;
    }

    @Override
    public boolean isFinished() {
        return currentBalanceState == BalanceState.Complete;
    }

    @Override
    public void end(boolean interrupted) {
        log.info("Ending.");
        drive.setVelocityMaintainerXTarget(0);
        drive.setActivateBrakeOverride(false);
    }
}
