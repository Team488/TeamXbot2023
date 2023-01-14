package competition.subsystems.drive.commands;

import javax.inject.Inject;

import competition.subsystems.drive.DriveSubsystem;
import xbot.common.command.BaseCommand;

/**
 * Changes the active swerve module when debugging. Meant to be used with {@link DebuggingSwerveWithJoysticksCommand}
 */
public class GoToNextActiveSwerveModuleCommand extends BaseCommand {

    DriveSubsystem drive;

    @Inject
    public GoToNextActiveSwerveModuleCommand(DriveSubsystem drive) {
        this.drive = drive;
    }

    @Override
    public void initialize() {
        log.info("Initializing");
        drive.setNextModuleAsActiveModule();
    }

    @Override
    public void execute() {
        // Nothing to do.
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}