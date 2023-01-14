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
 * A really basic crab drive command that sets raw powers to the swerve modules.
 * As a result,
 * the wheels go out of alignment very quickly. Only used for extremely basic
 * benchtop testing,
 * and the {@link DebuggingSwerveWithJoysticksCommand} is probably a better
 * choice in most scenarios.
 */
public class SimpleCrabDriveFromGamepadCommand extends BaseCommand {

    final DriveSubsystem drive;
    final OperatorInterface oi;

    @Inject
    public SimpleCrabDriveFromGamepadCommand(
            DriveSubsystem drive,
            OperatorInterface oi,
            @FrontLeftDrive SwerveComponent frontLeft,
            @FrontRightDrive SwerveComponent frontRight,
            @RearLeftDrive SwerveComponent rearLeft,
            @RearRightDrive SwerveComponent rearRight) {
        this.drive = drive;
        this.oi = oi;

        this.addRequirements(drive,
                frontLeft.swerveDriveSubsystem(), frontRight.swerveDriveSubsystem(), rearLeft.swerveDriveSubsystem(),
                rearRight.swerveDriveSubsystem(),
                frontLeft.swerveSteeringSubsystem(), frontRight.swerveSteeringSubsystem(),
                rearLeft.swerveSteeringSubsystem(), rearRight.swerveSteeringSubsystem());
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

        drive.crabDrive(drivePower, steeringPower);
    }

}
