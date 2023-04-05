package competition.subsystems.vision;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import org.junit.Test;

import competition.BaseCompetitionTest;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import java.util.Arrays;

public class VisionSubsystemTest extends BaseCompetitionTest {

    @Test
    public void testIsEstimatedPoseReliableInvalidFiducialId() {
        VisionSubsystem subsystem = getInjectorComponent().visionSubsystem();
        assertFalse(subsystem.isEstimatedPoseReliable(new EstimatedRobotPose(
                new Pose3d(),
                0,
                Arrays.asList(
                        createTargetWithId(32))
        ), new Pose2d()));
    }

    @Test
    public void testIsEstimatedPoseReliableAtLeastOneInvalidFiducialId() {
        VisionSubsystem subsystem = getInjectorComponent().visionSubsystem();
        assertFalse(subsystem.isEstimatedPoseReliable(new EstimatedRobotPose(
                new Pose3d(),
                0,
                Arrays.asList(
                        createTargetWithId(1),
                        createTargetWithId(8),
                        createTargetWithId(32))
        ), new Pose2d()));
    }

    @Test
    public void testIsEstimatedPositionReliable() {
        VisionSubsystem subsystem = getInjectorComponent().visionSubsystem();
        assertFalse(subsystem.isEstimatedPoseReliable(new EstimatedRobotPose(
                new Pose3d(),
                0,
                Arrays.asList(
                        createTargetWithId(1))
        ), new Pose2d(200,200,new Rotation2d())));
    }

    private PhotonTrackedTarget createTargetWithId(int id) {
        return new PhotonTrackedTarget(0, 0, 0, 0,
                id,
                new Transform3d(),
                new Transform3d(),
                0,
                Arrays.asList(
                        new TargetCorner(0, 0),
                        new TargetCorner(0, 0),
                        new TargetCorner(0, 0),
                        new TargetCorner(0, 0)),
                Arrays.asList(
                        new TargetCorner(0, 0),
                        new TargetCorner(0, 0),
                        new TargetCorner(0, 0),
                        new TargetCorner(0, 0)));
    }

}
