package competition.subsystems.drive.commands;

import javax.inject.Inject;

import competition.injection.swerve.FrontLeftDrive;
import competition.injection.swerve.FrontRightDrive;
import competition.injection.swerve.RearLeftDrive;
import competition.injection.swerve.RearRightDrive;
import competition.injection.swerve.SwerveComponent;
import competition.operator_interface.OperatorInterface;
import competition.subsystems.drive.DriveSubsystem;
import xbot.common.command.BaseCommand;
import xbot.common.math.MathUtils;

/**
 * Special debug command that takes control of all the swerve modules, but only
 * controls one
 * at a time while having all the others stay stopped. Useful for testing
 * individual modules
 * without the distraction of having all the other ones run.
 */
public class DebuggingSwerveWithJoysticksCommand extends BaseCommand {

    final DriveSubsystem drive;
    final OperatorInterface oi;

    @Inject
    public DebuggingSwerveWithJoysticksCommand(
            DriveSubsystem drive,
            OperatorInterface oi,
            @FrontLeftDrive SwerveComponent frontLeftSwerve,
            @FrontRightDrive SwerveComponent frontRightSwerve,
            @RearLeftDrive SwerveComponent rearLeftSwerve,
            @RearRightDrive SwerveComponent rearRightSwerve) {
        this.drive = drive;
        this.oi = oi;

        this.addRequirements(drive,
                frontLeftSwerve.swerveDriveSubsystem(), frontRightSwerve.swerveDriveSubsystem(),
                rearLeftSwerve.swerveDriveSubsystem(), rearRightSwerve.swerveDriveSubsystem(),
                frontLeftSwerve.swerveSteeringSubsystem(), frontRightSwerve.swerveSteeringSubsystem(),
                rearLeftSwerve.swerveSteeringSubsystem(), rearRightSwerve.swerveSteeringSubsystem());
    }

    @Override
    public void initialize() {
        log.info("Initializing");
    }

    @Override
    public void execute() {
        double drivePower = MathUtils.deadband(oi.driverGamepad.getLeftStickY(), oi.getDriverGamepadTypicalDeadband(),
                (a) -> a);
        double steeringPower = MathUtils.deadband(oi.driverGamepad.getRightStickX(),
                oi.getDriverGamepadTypicalDeadband(), (a) -> a);

        drive.controlOnlyActiveSwerveModuleSubsystem(drivePower, steeringPower);
    }

}
