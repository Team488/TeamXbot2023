package competition.subsystems.drive.commands;

import competition.subsystems.drive.DriveSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class ManualBalanceModeCommand extends BaseCommand {

    private final DriveSubsystem drive;

    @Inject
    public ManualBalanceModeCommand(DriveSubsystem drive) {
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() { log.info("Initializing"); }

    @Override
    public void execute() {
        drive.setManualBalanceMode(!drive.isManualBalanceModeActive());
    }
}
