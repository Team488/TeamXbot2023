package competition.subsystems.drive.commands;

import javax.inject.Inject;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import xbot.common.command.BaseCommand;
import xbot.common.math.PIDManager;
import xbot.common.math.XYPair;
import xbot.common.math.PIDManager.PIDManagerFactory;

public class AutoBalanceCommand extends BaseCommand {
    
    private final DriveSubsystem drive;
    private final PoseSubsystem pose;
    private final PIDManager pidManager;

    private boolean drivingAgainstPositiveAngle = true;
    final double firstAttemptSpeed = 0.25;
    double currentAttemptSpeed = firstAttemptSpeed;

    public enum BalanceState {
        Driving,
        Locked,
        Complete
    }

    private BalanceState currentBalanceState = BalanceState.Driving;

    @Inject
    public AutoBalanceCommand(DriveSubsystem drive, PoseSubsystem pose, PIDManagerFactory pidFactory) {
        this.drive = drive;
        this.pose = pose;
        this.pidManager = pidFactory.create(this.getPrefix(), 0.1, 0, 0);
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
        currentAttemptSpeed = firstAttemptSpeed;
        currentBalanceState = BalanceState.Driving;
    }

    @Override
    public void execute() {
        // Let's assume that we are already strongly "on the ramp", which is to say, at or very close
        // to the resting angle of the robot, which is +/-15 degrees.

        switch (currentBalanceState) {
            case Driving:
                break;
            case Locked:
                drive
                break;
            case Complete:
                return;
        }


        double currentAngle = getAngle();
        double velocity = pidManager.calculate(0, currentAngle);

        drive.setVelocityMaintainerXTarget(velocity);
    }

    private double getAngle() {
        // on the 2023 robot it's roll
        // on the 2022 robot it's pitch
        // this is based on the rio orientation
        double currentAngle = pose.getRobotRoll();

        if (Math.abs(currentAngle) < 1.5) {
            currentAngle = 0;
        }

        return currentAngle;
    }

    @Override
    public boolean isFinished() {
        return currentBalanceState == BalanceState.Complete;
    }

    @Override
    public void end(boolean interrupted) {
        drive.setVelocityMaintainerXTarget(0);
    }
}
