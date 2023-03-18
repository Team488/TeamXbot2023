package competition.subsystems.drive.commands;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import xbot.common.command.BaseCommand;
import xbot.common.math.XYPair;
import xbot.common.subsystems.drive.control_logic.HeadingModule;

import javax.inject.Inject;

public class TurnLeft180DegreesCommand extends BaseCommand {
    private Rotation2d initialHeading;
    private Rotation2d goalHeading;
    DriveSubsystem drive;
    PoseSubsystem pose;
    HeadingModule headingModule;
    @Inject
    public TurnLeft180DegreesCommand(DriveSubsystem drive, PoseSubsystem pose, HeadingModule.HeadingModuleFactory headingModuleFactory){
        this.drive = drive;
        this.pose = pose;
        this.headingModule = headingModuleFactory.create(drive.getRotateToHeadingPid());

    }

    @Override
    public void initialize() {
        log.info("Initializing");
        initialHeading = pose.getCurrentHeading();
        goalHeading = initialHeading.plus(Rotation2d.fromDegrees(180));
    }

    @Override
    public void execute() {
        double power = headingModule.calculateHeadingPower(goalHeading);
        drive.move(new XYPair(),power);
    }

    @Override
    public boolean isFinished(){
        return headingModule.isOnTarget();
    }
}
