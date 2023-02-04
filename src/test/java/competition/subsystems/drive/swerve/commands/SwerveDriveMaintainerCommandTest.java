package competition.subsystems.drive.swerve.commands;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import competition.BaseCompetitionTest;
import competition.subsystems.drive.commands.SwerveDriveMaintainerCommand;
import competition.subsystems.drive.swerve.SwerveDriveSubsystem;

public class SwerveDriveMaintainerCommandTest extends BaseCompetitionTest {
    
    SwerveDriveMaintainerCommand command;
    SwerveDriveSubsystem subsystem;

    @Override
    public void setUp() {
        super.setUp();
        subsystem = getInjectorComponent().swerveComponents().frontLeft.swerveDriveSubsystem();
        command = getInjectorComponent().swerveComponents().frontLeft.swerveDriveMaintainerCommand();
    }

    @Test
    public void testMaintain() {
        subsystem.setTargetValue(1.0);
        
        command.initialize();
        command.execute();

        assertTrue("Wheel should be rotating clockwise", subsystem.getSparkMax().get() < 0.1);
    }

}
