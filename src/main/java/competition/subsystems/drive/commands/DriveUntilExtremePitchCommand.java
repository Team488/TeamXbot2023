package competition.subsystems.drive.commands;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import xbot.common.command.BaseCommand;
import xbot.common.math.XYPair;

import javax.inject.Inject;
import java.util.function.Supplier;

public class DriveUntilExtremePitchCommand extends BaseCommand {

    final DriveSubsystem drive;
    final PoseSubsystem pose;

    double drivePower = 0.75;

    public enum BlueAllianceDriveDirection {
        East,
        West
    }

    private BlueAllianceDriveDirection fieldOrientedDriveDirection;
    private Supplier<BlueAllianceDriveDirection> driveDirectionSupplier;

    @Inject
    public DriveUntilExtremePitchCommand(DriveSubsystem drive, PoseSubsystem pose) {
        this.drive = drive;
        this.pose = pose;
    }

    public void setDriveDirectionSupplier(Supplier<BlueAllianceDriveDirection> directionSupplier) {
        this.driveDirectionSupplier = directionSupplier;
    }

    public void setFieldOrientedDriveDirection(BlueAllianceDriveDirection direction) {
        this.driveDirectionSupplier = () -> direction;
    }

    public void setDrivePower(double power) {
        drivePower = power;
    }

    @Override
    public void initialize() {
        log.info("Initializing");

        fieldOrientedDriveDirection = driveDirectionSupplier.get();

        // Check if we need to mirror due to being on red alliance.
        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            if (fieldOrientedDriveDirection == BlueAllianceDriveDirection.East) {
                fieldOrientedDriveDirection = BlueAllianceDriveDirection.West;
            } else {
                fieldOrientedDriveDirection = BlueAllianceDriveDirection.East;
            }
        }
    }

    @Override
    public void execute() {
        XYPair translationIntent = new XYPair();
        if (fieldOrientedDriveDirection == BlueAllianceDriveDirection.East) {
            translationIntent = new XYPair(drivePower, 0);
        }
        else if (fieldOrientedDriveDirection == BlueAllianceDriveDirection.West) {
            translationIntent = new XYPair(-drivePower, 0);
        }

        drive.fieldOrientedDrive(
                translationIntent,
                0.0,
                pose.getCurrentHeading().getDegrees(),
                false);
    }

    @Override
    public boolean isFinished() {
        return pose.getHasExperiencedExtremePitch();
    }
}
