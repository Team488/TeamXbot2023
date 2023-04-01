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
import xbot.common.properties.Property;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;
import java.io.IOException;
import java.util.Optional;

public class VisionSubsystem extends BaseSubsystem {

    public static final String VISION_TABLE = "photonvision";

    public static final String TARGET_POSE = "forwardAprilCamera/targetPose";
    public static final String LATENCY_MILLIS = "forwardAprilCamera/latencyMillis";

    final PhotonCamera forwardAprilCamera;
    final PhotonCamera rearAprilCamera;

    final RobotAssertionManager assertionManager;
    final BooleanProperty isInverted;
    final DoubleProperty yawOffset;
    final DoubleProperty waitForStablePoseTime;
    final TimeStableValidator frontReliablePoseIsStable;
    final TimeStableValidator rearReliablePoseIsStable;
    NetworkTable visionTable;
    AprilTagFieldLayout aprilTagFieldLayout;
    XbotPhotonPoseEstimator customPhotonPoseEstimator;
    PhotonPoseEstimator photonPoseEstimator;
    PhotonPoseEstimator rearPhotonPoseEstimator;
    boolean visionWorking = false;


    @Inject
    public VisionSubsystem(PropertyFactory pf, RobotAssertionManager assertionManager) {
        this.assertionManager = assertionManager;
        visionTable = NetworkTableInstance.getDefault().getTable(VISION_TABLE);

        pf.setPrefix(this);
        isInverted = pf.createPersistentProperty("Yaw inverted", true);
        yawOffset = pf.createPersistentProperty("Yaw offset", 0);

        waitForStablePoseTime = pf.createPersistentProperty("Pose stable time", 0.25, Property.PropertyLevel.Debug);
        frontReliablePoseIsStable = new TimeStableValidator(() -> waitForStablePoseTime.get());
        rearReliablePoseIsStable = new TimeStableValidator(() -> waitForStablePoseTime.get());

        // TODO: Add resiliency to this subsystem, so that if the camera is not connected, it doesn't cause a pile
        // of errors. Some sort of VisionReady in the ElectricalContract may also make sense. Similarly,
        // we need to handle cases like not having the AprilTag data loaded.

        forwardAprilCamera = new PhotonCamera("forwardAprilCamera");
        rearAprilCamera = new PhotonCamera("rearAprilCamera");

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
                new Rotation3d(0, 0, Math.toRadians(7.595)));
        Transform3d robotToRearCam = new Transform3d(new Translation3d(
                -1.395 / PoseSubsystem.INCHES_IN_A_METER,
                11.712 / PoseSubsystem.INCHES_IN_A_METER,
                16.421 / PoseSubsystem.INCHES_IN_A_METER),
                new Rotation3d(0, 0, Math.toRadians(180 + 7.595)));
        customPhotonPoseEstimator = new XbotPhotonPoseEstimator(
            aprilTagFieldLayout, 
            XbotPhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
            forwardAprilCamera, 
            robotToCam);
        customPhotonPoseEstimator.setMaximumPoseAmbiguityThreshold(0.2);
        photonPoseEstimator = new PhotonPoseEstimator(
                aprilTagFieldLayout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP,
                forwardAprilCamera,
                robotToCam
        );
        rearPhotonPoseEstimator = new PhotonPoseEstimator(
                aprilTagFieldLayout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP,
                rearAprilCamera,
                robotToRearCam
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
            var isReliable = !estimatedPose.isEmpty() && isEstimatedPoseReliable(estimatedPose.get());
            var isStable = frontReliablePoseIsStable.checkStable(isReliable);
            if (isReliable && isStable) {
                return estimatedPose;
            }
            return Optional.empty();
        } else {
            return Optional.empty();
        }
    }

    public Optional<EstimatedRobotPose> getRearPhotonVisionEstimatedPose(Pose2d previousEstimatedRobotPose) {
        if (visionWorking) {
            rearPhotonPoseEstimator.setReferencePose(previousEstimatedRobotPose);
            var estimatedPose = rearPhotonPoseEstimator.update();
            var isReliable = !estimatedPose.isEmpty() && isEstimatedPoseReliable(estimatedPose.get());
            var isStable = rearReliablePoseIsStable.checkStable(isReliable);
            if (isReliable && isStable) {
                return estimatedPose;
            }
            return Optional.empty();
        } else {
            return Optional.empty();
        }
    }

    public boolean isEstimatedPoseReliable(EstimatedRobotPose estimatedPose) {
        if (estimatedPose.targetsUsed.size() == 0) {
            return false;
        }

        // Pose isn't reliable if we see a tag id that shouldn't be on the field
        var allTagIds = estimatedPose.targetsUsed.stream()
                .map(target -> target.getFiducialId()).toList();
        if (allTagIds.stream().anyMatch(id -> id < 1 || id > 8)) {
            this.log.warn("Ignoring vision pose with invalid tag id. Visible tags: "
                    + String.join(", ", allTagIds.stream().mapToInt(id -> id).mapToObj(id -> Integer.toString(id)).toList()));
            return false;
        }

        // Two or more targets tends to be very reliable
        if (estimatedPose.targetsUsed.size() > 1) {
            return true;
        }

        // For a single target we need to be above reliability threshold and within 1m
        return estimatedPose.targetsUsed.get(0).getPoseAmbiguity() < customPhotonPoseEstimator.getMaximumPoseAmbiguityThreshold()
                && estimatedPose.targetsUsed.get(0).getBestCameraToTarget().getTranslation().getX() < 1.5;
    }
}
