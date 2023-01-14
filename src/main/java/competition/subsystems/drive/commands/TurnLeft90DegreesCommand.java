package competition.subsystems.drive.commands;

import javax.inject.Inject;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import xbot.common.command.BaseCommand;
import xbot.common.math.XYPair;
import xbot.common.subsystems.drive.control_logic.HeadingModule;
import xbot.common.subsystems.drive.control_logic.HeadingModule.HeadingModuleFactory;

/**
 * Simple command to help us tune the drive's rotation PID.
 */
public class TurnLeft90DegreesCommand extends BaseCommand {

    private Rotation2d initialHeading;
    private Rotation2d goalHeading;
    PoseSubsystem pose;
    DriveSubsystem drive;
    HeadingModule headingModule;

    @Inject
    public TurnLeft90DegreesCommand(DriveSubsystem drive, PoseSubsystem pose, HeadingModuleFactory headingModuleFactory) {
        this.addRequirements(drive);
        
        this.drive = drive;
        this.pose = pose;
        this.headingModule = headingModuleFactory.create(drive.getRotateToHeadingPid());
    }

    @Override
    public void initialize() {
        log.info("Initializing");
        initialHeading = pose.getCurrentHeading();
        goalHeading = initialHeading.plus(Rotation2d.fromDegrees(90));
    }

    @Override
    public void execute() {
        double power = headingModule.calculateHeadingPower(goalHeading);
        // Just rotate.
        drive.move(new XYPair(), power);
    }

    @Override
    public boolean isFinished() {
        return headingModule.isOnTarget();
    }
}