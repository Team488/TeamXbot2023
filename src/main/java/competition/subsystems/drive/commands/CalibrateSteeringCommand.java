package competition.subsystems.drive.commands;

import javax.inject.Inject;

import competition.injection.swerve.FrontLeftDrive;
import competition.injection.swerve.FrontRightDrive;
import competition.injection.swerve.RearLeftDrive;
import competition.injection.swerve.RearRightDrive;
import competition.injection.swerve.SwerveComponent;
import competition.subsystems.drive.swerve.SwerveSteeringSubsystem;
import xbot.common.command.BaseCommand;

/**
 * Used to force all the swerve steering modules to calibrate at their current position.
 * Make sure that the wheels are all facing forward before running this command!
 * You should also never need to run this command once we have the CANCoders active, since they
 * should be able to always read the true, absolute position of the steering modules.
 */
public class CalibrateSteeringCommand extends BaseCommand {

    private final SwerveSteeringSubsystem frontLeft;
    private final SwerveSteeringSubsystem frontRight;
    private final SwerveSteeringSubsystem rearLeft;
    private final SwerveSteeringSubsystem rearRight;

    @Inject
    public CalibrateSteeringCommand(
        @FrontLeftDrive SwerveComponent frontLeftSteering,
        @FrontRightDrive SwerveComponent frontRightSteering,
        @RearLeftDrive SwerveComponent rearLeftSteering,
        @RearRightDrive SwerveComponent rearRightSteering) {

        this.frontLeft = frontLeftSteering.swerveSteeringSubsystem();
        this.frontRight = frontRightSteering.swerveSteeringSubsystem();
        this.rearLeft = rearLeftSteering.swerveSteeringSubsystem();
        this.rearRight = rearRightSteering.swerveSteeringSubsystem();
    }

    @Override
    public void initialize() {
        log.info("Initializing");

        frontLeft.calibrateHere();
        frontRight.calibrateHere();
        rearLeft.calibrateHere();
        rearRight.calibrateHere();
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}