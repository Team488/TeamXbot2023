package competition.subsystems.drive.swerve.commands;

import competition.BaseCompetitionTest;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.drive.commands.ManualBalanceModeCommand;
import org.junit.Test;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class ManualBalanceModeCommandTest extends BaseCompetitionTest {

    @Test
    public void testToggle() {
        DriveSubsystem drive = (DriveSubsystem) getInjectorComponent().driveSubsystem();
        ManualBalanceModeCommand command = new ManualBalanceModeCommand(drive);

        assertFalse(drive.isManualBalanceModeActive());

        command.initialize();

        assertTrue(drive.isManualBalanceModeActive());

        command.initialize();

        assertFalse(drive.isManualBalanceModeActive());
    }

}
