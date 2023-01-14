package competition.subsystems.drive.commands;

import javax.inject.Inject;

import competition.subsystems.drive.DriveSubsystem;
import xbot.common.command.BaseCommand;
import xbot.common.math.XYPair;

public class StopDriveCommand extends BaseCommand {
    
    private final DriveSubsystem drive;

    @Inject
    public StopDriveCommand(DriveSubsystem drive) {
        this.drive = drive;
    }

    @Override
    public void initialize() {
        log.info("Initializing");
    }

    @Override
    public void execute() {
        drive.move(new XYPair(0,0), 0);
    }
}
