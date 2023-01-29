package competition.subsystems.vision;

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
    public static final String FIX_ACQUIRED_PROPERTY = "gloworm/hasTarget";
    public static final String TARGET_YAW_PROPERTY = "gloworm/targetYaw";
    public static final String TARGET_PITCH_PROPERTY = "gloworm/targetPitch";

    public static final String TARGET_POSE = "TemporaryCam/targetPose";
    public static final String LATENCY_MILLIS = "TemporaryCam/latencyMillis";

    final PhotonCamera forwardAprilCamera;

    final RobotAssertionManager assertionManager;
    final BooleanProperty isInverted;
    final DoubleProperty yawOffset;
    final DoubleProperty waitForStableFixTime;
    final TimeStableValidator fixIsStable;
    NetworkTable visionTable;
    AprilTagFieldLayout aprilTagFieldLayout;
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

        //Cam mounted half a meter forward of center, half a meter up from center, and facing forward.
        Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));
        photonPoseEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout, 
            PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE, 
            forwardAprilCamera, 
            robotToCam);
    }

    public double getBearingToHub() {
        if (getFixAcquired()) {
            return getYawToTarget();
        } else {
            return 0;
        }
    }

    public double getPitchToHub() {
        if (getFixAcquired()) {
            return getPitchToTarget();
        } else {
            return 0;
        }
    }

    public boolean getFixAcquired() {
        boolean fixAcquired = visionTable.getEntry(FIX_ACQUIRED_PROPERTY).getBoolean(false);
        boolean isStable = fixIsStable.checkStable(fixAcquired);

        return fixAcquired && isStable;
    }

    private double getYawToTarget() {
        double yawToTarget = (visionTable.getEntry(TARGET_YAW_PROPERTY).getDouble(0) + this.yawOffset.get()) * getInversionFactor();

        return yawToTarget;
    }

    private double getPitchToTarget() {
        return (visionTable.getEntry(TARGET_PITCH_PROPERTY).getDouble(0));
    }

    private double getInversionFactor() {
        if (this.isInverted.get()) {
            return -1;
        }
        return 1;
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
            photonPoseEstimator.setReferencePose(previousEstimatedRobotPose);
            return photonPoseEstimator.update();
        } else {
            return Optional.empty();
        }
    }
}
