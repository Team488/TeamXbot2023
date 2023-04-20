package competition.subsystems.pose;

import competition.BaseCompetitionTest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import org.junit.Test;
import xbot.common.math.XYPair;

import static org.junit.Assert.assertEquals;

public class PoseSubsystemTest extends BaseCompetitionTest {

    MockPoseSubsystem pose;

    @Override
    public void setUp() {
        super.setUp();
        this.pose = (MockPoseSubsystem)this.getInjectorComponent().poseSubsystem();
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

    @Test
    public void testAllianceAwareTransformations() {
        pose.setAllianceAwareField(false);

        pose.setAlliance(DriverStation.Alliance.Blue);
        assertEquals(Rotation2d.fromDegrees(180), pose.rotateAngleBasedOnAlliance(Rotation2d.fromDegrees(180)));
        assertEquals(new XYPair(1, 1).toString(), pose.rotateVectorBasedOnAlliance(new XYPair(1, 1)).toString());

        pose.setAlliance(DriverStation.Alliance.Red);
        assertEquals(Rotation2d.fromDegrees(180), pose.rotateAngleBasedOnAlliance(Rotation2d.fromDegrees(180)));
        assertEquals(new XYPair(1, 1).toString(), pose.rotateVectorBasedOnAlliance(new XYPair(1, 1)).toString());

        pose.setAllianceAwareField(true);

        pose.setAlliance(DriverStation.Alliance.Blue);
        assertEquals(Rotation2d.fromDegrees(180), pose.rotateAngleBasedOnAlliance(Rotation2d.fromDegrees(180)));
        assertEquals(new XYPair(1, 1).toString(), pose.rotateVectorBasedOnAlliance(new XYPair(1, 1)).toString());

        pose.setAlliance(DriverStation.Alliance.Red);
        assertEquals(Rotation2d.fromDegrees(0), pose.rotateAngleBasedOnAlliance(Rotation2d.fromDegrees(180)));
        assertEquals(new XYPair(-1, -1).toString(), pose.rotateVectorBasedOnAlliance(new XYPair(1, 1)).toString());
    }
}
