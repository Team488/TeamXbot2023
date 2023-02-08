package competition.subsystems.pose;

import competition.BaseCompetitionTest;
import competition.subsystems.drive.DriveSubsystem;
import org.junit.Test;

public class PoseSubsystemTest extends BaseCompetitionTest {

    PoseSubsystem pose;
    DriveSubsystem drive;

    @Override
    public void setUp() {
        super.setUp();
        this.pose = (PoseSubsystem)this.getInjectorComponent().poseSubsystem();
        this.drive = (DriveSubsystem)this.getInjectorComponent().driveSubsystem();
        drive.refreshDataFrame();
    }

    @Test
    public void simpleTest() {
        // Quick test for any instant runtime issues with new methods
        pose.updateOdometry();
        pose.getIsPoseHealthy();
        pose.getIsVisionPoseExtremelyConfident();
        pose.getVisionAssistedPositionInMeters();
        pose.setCurrentPoseInMeters(pose.getVisionAssistedPositionInMeters());
    }
}
