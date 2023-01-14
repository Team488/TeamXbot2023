package competition.subsystems.drive.swerve;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import competition.BaseCompetitionTest;
import competition.subsystems.drive.DriveSubsystem;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModuleSubsystemTest extends BaseCompetitionTest {
    
    @Test
    public void testDefaultTargetValue() {
        SwerveModuleSubsystem swerveModuleSubsystem = ((DriveSubsystem)getInjectorComponent().driveSubsystem())
            .getFrontLeftSwerveModuleSubsystem();

        SwerveModuleState state = swerveModuleSubsystem.getTargetState();

        assertEquals(0, state.angle.getDegrees(), 0.001);
        assertEquals(0, state.speedMetersPerSecond, 0.001);
    }

}
