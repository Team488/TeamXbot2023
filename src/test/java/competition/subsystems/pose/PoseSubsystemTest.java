package competition.subsystems.pose;

import competition.BaseCompetitionTest;
import org.junit.Test;

public class PoseSubsystemTest extends BaseCompetitionTest {

    PoseSubsystem pose;

    @Override
    public void setUp() {
        super.setUp();
        this.pose = (PoseSubsystem)this.getInjectorComponent().poseSubsystem();
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
