package competition.subsystems.drive.swerve.commands;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import competition.BaseCompetitionTest;
import competition.subsystems.drive.commands.SwerveSteeringMaintainerCommand;
import competition.subsystems.drive.swerve.SwerveSteeringSubsystem;

public class SwerveSteeringMaintainerCommandTest extends BaseCompetitionTest {
    
    SwerveSteeringMaintainerCommand command;
    SwerveSteeringSubsystem subsystem;

    @Override
    public void setUp() {
        super.setUp();
        subsystem = getInjectorComponent().swerveComponents().frontLeft.swerveSteeringSubsystem();
        command = getInjectorComponent().swerveComponents().frontLeft.swerveSteeringMaintainerCommand();
    }

    @Test
    public void testMaintain() {
        subsystem.setTargetValue(45);
        
        command.initialize();
        command.execute();

        assertTrue("Wheel should be rotating counter-clockwise", subsystem.getSparkMax().get() < 0.1);
    }

}
