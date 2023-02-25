package competition.subsystems.drive.commands;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import xbot.common.command.BaseCommand;
import xbot.common.math.PIDManager;
import xbot.common.math.XYPair;

import javax.inject.Inject;

public class BrakeCommand extends BaseCommand {

    private final DriveSubsystem drive;


    @Inject
    public BrakeCommand(DriveSubsystem drive) {
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        drive.setWheelsToXMode();
    }
}
