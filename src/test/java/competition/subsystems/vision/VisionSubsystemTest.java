package competition.subsystems.vision;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import competition.BaseCompetitionTest;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSubsystemTest extends BaseCompetitionTest {
    
    @Test
    public void testWhenTableEmpty() {
        VisionSubsystem subsystem = getInjectorComponent().visionSubsystem();
        NetworkTable visionTable = NetworkTableInstance.getDefault().getTable(VisionSubsystem.VISION_TABLE);
        assertEquals("Table should be empty", 0, visionTable.getKeys().size());

        assertEquals("Subsystem should not report a fix", false, subsystem.getFixAcquired());
        assertEquals("Target yaw should be 0", 0, subsystem.getBearingToHub(), 0.001);
    }

    @Test
    public void testGetFixAcquiredIsTimeStable() {
        VisionSubsystem subsystem = getInjectorComponent().visionSubsystem();
        NetworkTable visionTable = NetworkTableInstance.getDefault().getTable(VisionSubsystem.VISION_TABLE);
        assertEquals("Table should be empty", 0, visionTable.getKeys().size());

        assertEquals("Subsystem should not report a fix", false, subsystem.getFixAcquired());

        visionTable.getEntry(VisionSubsystem.FIX_ACQUIRED_PROPERTY).setBoolean(true);
        
        assertEquals("Subsystem should not report a fix, it's not stable yet", false, subsystem.getFixAcquired());

        timer.advanceTimeInSecondsBy(1);

        assertEquals("Subsystem should report a stable fix", true, subsystem.getFixAcquired());
    }

    @Test
    public void testTargetYawZeroWhileNoFix() {
        VisionSubsystem subsystem = getInjectorComponent().visionSubsystem();
        NetworkTable visionTable = NetworkTableInstance.getDefault().getTable(VisionSubsystem.VISION_TABLE);
        assertEquals("Table should be empty", 0, visionTable.getKeys().size());
        visionTable.getEntry(VisionSubsystem.FIX_ACQUIRED_PROPERTY).setBoolean(false);
        visionTable.getEntry(VisionSubsystem.TARGET_YAW_PROPERTY).setDouble(5);
        
        assertEquals("Subsystem should not report a target yaw, there's no fix", 0, subsystem.getBearingToHub(), 0.001);
    }

}
