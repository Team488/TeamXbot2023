package competition.subsystems.drive.swerve.commands;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import competition.BaseCompetitionTest;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.drive.commands.SetSwerveMotorControllerPidParametersCommand;
import competition.subsystems.drive.swerve.SwerveSteeringMotorPidSubsystem;
import competition.subsystems.drive.swerve.SwerveSteeringSubsystem;

public class SetSteeringMotorControllerPidParametersCommandTest extends BaseCompetitionTest {
    
    private SetSwerveMotorControllerPidParametersCommand command;
    private SwerveSteeringMotorPidSubsystem configSubsystem;
    private DriveSubsystem driveSubsystem;

    @Override
    public void setUp() {
        super.setUp();
        this.command = getInjectorComponent().setSwerveMotorControllerPidParametersCommand();
        this.configSubsystem = getInjectorComponent().swerveSteeringMotorPidSubsystem();
        this.driveSubsystem = (DriveSubsystem)getInjectorComponent().driveSubsystem();
    }

    @Test
    public void testCommand() {

        this.configSubsystem.setAllProperties(1, 2, 3, 4, 5, 6, 7, 8);

        this.command.initialize();
        this.command.execute();

        assertTrue("Command should be finished", this.command.isFinished());

        SwerveSteeringSubsystem[] subsystems = {
            this.driveSubsystem.getFrontLeftSwerveModuleSubsystem().getSteeringSubsystem(),
            this.driveSubsystem.getFrontRightSwerveModuleSubsystem().getSteeringSubsystem(),
            this.driveSubsystem.getRearLeftSwerveModuleSubsystem().getSteeringSubsystem(),
            this.driveSubsystem.getRearRightSwerveModuleSubsystem().getSteeringSubsystem()
        };

        for (SwerveSteeringSubsystem subsystem : subsystems) {
            assertEquals("P should be set correctly", 1, subsystem.getSparkMax().getP(), 0.001);
            assertEquals("I should be set correctly", 2, subsystem.getSparkMax().getI(), 0.001);
            assertEquals("D should be set correctly", 3, subsystem.getSparkMax().getD(), 0.001);
            assertEquals("FF should be set correctly", 4, subsystem.getSparkMax().getFF(), 0.001);
            assertEquals("MinOutput should be set correctly", 5, subsystem.getSparkMax().getOutputMin(), 0.001);
            assertEquals("MaxOutput should be set correctly", 6, subsystem.getSparkMax().getOutputMax(), 0.001);
        }
    }
}
