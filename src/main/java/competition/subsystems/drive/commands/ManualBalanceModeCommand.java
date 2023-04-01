package competition.subsystems.drive.commands;

import competition.subsystems.drive.DriveSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class ManualBalanceModeCommand extends BaseCommand {

    private final DriveSubsystem drive;

    @Inject
    public ManualBalanceModeCommand(DriveSubsystem drive) {
        this.drive = drive;
    }

    @Override
    public void initialize() {
        boolean newMode = !drive.isManualBalanceModeActive();
        log.info("Manual balance mode: " + (newMode ? "enabled" : "disabled"));
        drive.setManualBalanceMode(newMode);
    }

    @Override
    public void execute() {
        // do nothing
    }

    @Override
    public boolean isFinished() {
        super.isFinished();
        return true;
    }
}
