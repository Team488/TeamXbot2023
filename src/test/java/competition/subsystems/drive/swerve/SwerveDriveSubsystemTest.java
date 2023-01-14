package competition.subsystems.drive.swerve;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import competition.BaseCompetitionTest;
import competition.subsystems.drive.DriveSubsystem;

public class SwerveDriveSubsystemTest extends BaseCompetitionTest {
    
    @Test
    public void testGetSetTargetValue() {
        SwerveDriveSubsystem swerveDriveSubsystem = ((DriveSubsystem)getInjectorComponent().driveSubsystem())
            .getFrontLeftSwerveModuleSubsystem()
            .getDriveSubsystem();
            
        assertEquals(swerveDriveSubsystem.getTargetValue(), 0.0, 0.001);

        swerveDriveSubsystem.setTargetValue(1.0);

        assertEquals(swerveDriveSubsystem.getTargetValue(), 1.0, 0.001);
    }
}
