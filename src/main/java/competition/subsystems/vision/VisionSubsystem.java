package competition.subsystems.vision;

import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.pose.XbotPhotonPoseEstimator;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import xbot.common.command.BaseSubsystem;
import xbot.common.logging.RobotAssertionManager;
import xbot.common.logic.TimeStableValidator;
import xbot.common.math.XYPair;
import xbot.common.properties.BooleanProperty;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;
import java.io.IOException;
import java.util.Optional;

public class VisionSubsystem extends BaseSubsystem {

    public static final String VISION_TABLE = "photonvision";

    public static final String TARGET_POSE = "forwardAprilCamera/targetPose";
    public static final String LATENCY_MILLIS = "forwardAprilCamera/latencyMillis";

    final PhotonCamera forwardAprilCamera;

    final RobotAssertionManager assertionManager;
    final BooleanProperty isInverted;
    final DoubleProperty yawOffset;
    final DoubleProperty waitForStableFixTime;
    final TimeStableValidator fixIsStable;
    NetworkTable visionTable;
    AprilTagFieldLayout aprilTagFieldLayout;
    XbotPhotonPoseEstimator customPhotonPoseEstimator;
    PhotonPoseEstimator photonPoseEstimator;
    boolean visionWorking = false;


    @Inject
    public VisionSubsystem(PropertyFactory pf, RobotAssertionManager assertionManager) {
        this.assertionManager = assertionManager;
        visionTable = NetworkTableInstance.getDefault().getTable(VISION_TABLE);

        pf.setPrefix(this);
        isInverted = pf.createPersistentProperty("Yaw inverted", true);
        yawOffset = pf.createPersistentProperty("Yaw offset", 0);

        waitForStableFixTime = pf.createPersistentProperty("Fix stable time", 0.1);
        fixIsStable = new TimeStableValidator(() -> waitForStableFixTime.get());

        // TODO: Add resiliency to this subsystem, so that if the camera is not connected, it doesn't cause a pile
        // of errors. Some sort of VisionReady in the ElectricalContract may also make sense. Similarly,
        // we need to handle cases like not having the AprilTag data loaded.

        forwardAprilCamera = new PhotonCamera("forwardAprilCamera");

        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
            visionWorking = true;
            log.info("Successfully loaded AprilTagFieldLayout");
        } catch (IOException e) {
            log.error("Could not load AprilTagFieldLayout!", e);
        }

        //Cam mounted 1" forward of center, 17" up, 12.5" right.
        Transform3d robotToCam = new Transform3d(new Translation3d(
                1.395 / PoseSubsystem.INCHES_IN_A_METER,
                -11.712 / PoseSubsystem.INCHES_IN_A_METER,
                16.421 / PoseSubsystem.INCHES_IN_A_METER),
                new Rotation3d(0, Math.toRadians(62.5), Math.toRadians(7.595)));
        customPhotonPoseEstimator = new XbotPhotonPoseEstimator(
            aprilTagFieldLayout, 
            XbotPhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
            forwardAprilCamera, 
            robotToCam);
        customPhotonPoseEstimator.setMaximumPoseAmbiguityThreshold(0.00001);
        photonPoseEstimator = new PhotonPoseEstimator(
                aprilTagFieldLayout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP,
                forwardAprilCamera,
                robotToCam
        );
    }

    public XYPair getAprilCoordinates() {

        var result = forwardAprilCamera.getLatestResult();
        if (!result.hasTargets()) {
            return null;
        }


        NetworkTable aprilTable = NetworkTableInstance.getDefault().getTable(VISION_TABLE);
        double[] targetPose = aprilTable.getEntry(TARGET_POSE).getDoubleArray(new double[0]);
        if (targetPose.length == 0) {
            return null;
        }
        if (Math.abs(targetPose[0]) < 0.01 && Math.abs(targetPose[1]) < 0.01) {
            return null;
        }
        return new XYPair(-targetPose[0], -targetPose[1]);
    }

    public Optional<EstimatedRobotPose> getPhotonVisionEstimatedPose(Pose2d previousEstimatedRobotPose) {
        if (visionWorking) {
            //customPhotonPoseEstimator.setReferencePose(previousEstimatedRobotPose);
            //return customPhotonPoseEstimator.update();
            photonPoseEstimator.setReferencePose(previousEstimatedRobotPose);
            var estimatedPose = photonPoseEstimator.update();
            if (!estimatedPose.isEmpty() && this.isEstimatedPoseReliable(estimatedPose.get())) {
                return estimatedPose;
            }
            return Optional.empty();
        } else {
            return Optional.empty();
        }
    }

    private boolean isEstimatedPoseReliable(EstimatedRobotPose estimatedPose) {
        if (estimatedPose.targetsUsed.size() == 0) {
            return false;
        }

        // Two or more targets tends to be very reliable
        if (estimatedPose.targetsUsed.size() > 1) {
            return true;
        }

        // For a single target we need to be above reliability threshold
        var singleTarget = estimatedPose.targetsUsed.get(0);
        return singleTarget.getPoseAmbiguity() < 0.001
                && singleTarget.getBestCameraToTarget().getTranslation().getX() < 1.5;
    }
}
