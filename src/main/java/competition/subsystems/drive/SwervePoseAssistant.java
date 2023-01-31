package competition.subsystems.drive;

import competition.subsystems.pose.PoseSubsystem;
import dagger.assisted.Assisted;
import dagger.assisted.AssistedFactory;
import dagger.assisted.AssistedInject;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.apache.log4j.Logger;
import org.photonvision.EstimatedRobotPose;
import xbot.common.controls.sensors.XGyro;
import xbot.common.controls.sensors.XTimer;
import xbot.common.logic.Latch;
import xbot.common.logic.TimeStableValidator;
import xbot.common.math.FieldPose;
import xbot.common.math.WrappedRotation2d;
import xbot.common.math.XYPair;
import xbot.common.properties.BooleanProperty;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.pose.BasePoseSubsystem;

import java.util.Optional;
import java.util.function.Supplier;

/**
 * General idea - pull Pose Calculations out of the PoseSubsystem and embed them here, which in turn will be embedded
 * in the DriveSubsystem. This will allow for clear ordering of data transformations in the robot.
 *
 * Still a prototype - for this year it probably makes more sense to keep using the PoseSubsystem, but eventually
 * we can clean up the relationship between these classes.
 */
public class SwervePoseAssistant {

    final SwerveDrivePoseEstimator swerveOdometry;
    private final Field2d fieldForDisplay;

    private TimeStableValidator healthyPoseValidator = new TimeStableValidator(1);
    private final DoubleProperty suprisingVisionUpdateDistanceInMetersProp;
    private final BooleanProperty isPoseHealthyProp;
    private TimeStableValidator extremelyConfidentVisionValidator = new TimeStableValidator(10);
    private final DoubleProperty extremelyConfidentVisionDistanceUpdateInMetersProp;
    private final BooleanProperty isVisionPoseExtremelyConfidentProp;
    private final Latch useVisionToUpdateGyroLatch;

    private Logger log;

    private Pose2d currentPoseWithGyroHeadingMetric;
    private Pose2d currentPoseWithVisionHeadingMetric;

    private XGyro gyro;

    double headingOffset;
    private double lastSetHeadingTime;

    private final DoubleProperty totalDistanceXinches;
    private final DoubleProperty totalDistanceYinches;

    Supplier<SwerveModulePosition[]> currentModulePositionsSupplier;

    @AssistedFactory
    public abstract static class SwervePoseAssistantFactory {
        public abstract SwervePoseAssistant create(
                @Assisted("kinematics") SwerveDriveKinematics kinematics,
                @Assisted("currentModulePositionsSupplier") Supplier<SwerveModulePosition[]> currentModulePositionsSupplier,
                @Assisted("gyro") XGyro gyro);
    }

    @AssistedInject
    public SwervePoseAssistant(
            @Assisted("kinematics") SwerveDriveKinematics kinematics,
            @Assisted("currentModulePositionsSupplier") Supplier<SwerveModulePosition[]> currentModulePositionsSupplier,
            @Assisted("gyro") XGyro gyro,
            PropertyFactory pf) {

        log = Logger.getLogger("SwervePoseAssistant");
        pf.setPrefix("SwervePoseAssistant");
        this.gyro = gyro;
        this.currentModulePositionsSupplier = currentModulePositionsSupplier;
        suprisingVisionUpdateDistanceInMetersProp = pf.createPersistentProperty("SuprisingVisionUpdateDistanceInMeters", 0.5);
        isPoseHealthyProp = pf.createEphemeralProperty("IsPoseHealthy", false);
        extremelyConfidentVisionDistanceUpdateInMetersProp = pf.createPersistentProperty("ExtremelyConfidentVisionDistanceUpdateInMeters", 0.01);
        isVisionPoseExtremelyConfidentProp = pf.createEphemeralProperty("IsVisionPoseExtremelyConfident", false);

        totalDistanceXinches = pf.createEphemeralProperty("Total distance X", 0.0);
        totalDistanceYinches = pf.createEphemeralProperty("Total distance Y", 0.0);

        fieldForDisplay = new Field2d();
        SmartDashboard.putData("Field", fieldForDisplay);

        swerveOdometry = new SwerveDrivePoseEstimator(
                kinematics,
                Rotation2d.fromDegrees(BasePoseSubsystem.FACING_AWAY_FROM_DRIVERS),
                currentModulePositionsSupplier.get(),
                new Pose2d());

        useVisionToUpdateGyroLatch = new Latch(false, Latch.EdgeType.RisingEdge, edge -> {
            if (edge== Latch.EdgeType.RisingEdge) {
                log.info("Vision has been so confident for so long that we are force-updating our overall pose.");
                this.setCurrentPoseInMeters(getCurrentPoseWithVisionHeadingMetric());
            }
        });
    }

    public WrappedRotation2d getCurrentHeading() {
        return WrappedRotation2d.fromDegrees(gyro.getHeading().getDegrees() + headingOffset);
    }


    public void setCurrentHeading(double headingInDegrees){
        log.info("Forcing heading to: " + headingInDegrees);
        double rawHeading = gyro.getHeading().getDegrees();
        log.info("Raw heading is: " + rawHeading);
        headingOffset = -rawHeading + headingInDegrees;
        log.info("Offset calculated to be: " + headingOffset);

        lastSetHeadingTime = XTimer.getFPGATimestamp();
    }

    public void processOdometry() {
        currentPoseWithGyroHeadingMetric = swerveOdometry.update(
                gyro.getHeading(),
                currentModulePositionsSupplier.get());

        //improveOdometryUsingSimpleAprilTag();
    }

    private void improveOdometryUsingSimpleAprilTag(Translation2d aprilCoordinates) {
        // Try to get some vision sauce in there
        // and feed it straight into the odometry, then do the shifting at the very end when we convert back to inches.
        if (aprilCoordinates != null) {
            Pose2d aprilPos = new Pose2d(aprilCoordinates.getX(), aprilCoordinates.getY(), getCurrentHeading());
            swerveOdometry.addVisionMeasurement(aprilPos, XTimer.getFPGATimestamp() - 0.030);
        }
    }

    private void improveOdometryUsingPhotonLib(Optional<EstimatedRobotPose> photonEstimatedPose) {

        if (photonEstimatedPose.isPresent()) {
            // Get the result data, which has both coordinates and timestamps
            var camPose = photonEstimatedPose.get();

            // Check for the distance delta between the current and suggested poses. If it's large, reset
            // the healthy pose validator.
            double distance = currentPoseWithGyroHeadingMetric.getTranslation().getDistance(
                    camPose.estimatedPose.toPose2d().getTranslation());

            boolean isSurprisingDistance = (distance > suprisingVisionUpdateDistanceInMetersProp.get());
            isPoseHealthyProp.set(healthyPoseValidator.checkStable(!isSurprisingDistance));

            // If the distance change is minuscule, we're extremely confident in the vision data, and
            // could consider using it to update the gyro.
            boolean isExtremelyConfident = (distance < extremelyConfidentVisionDistanceUpdateInMetersProp.get());
            isVisionPoseExtremelyConfidentProp.set(extremelyConfidentVisionValidator.checkStable(isExtremelyConfident));

            // In any case, update the odometry with the new pose from the camera.
            swerveOdometry.addVisionMeasurement(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
        } {
            // Since we didn't get any vision updates, we assume the current pose is healthy.
            isPoseHealthyProp.set(healthyPoseValidator.checkStable(true));
            // But since we didn't get any vision updates, we can't be super-confident!
            isVisionPoseExtremelyConfidentProp.set(extremelyConfidentVisionValidator.checkStable(false));
        }
    }

    public void setCurrentPositionInInches(double newXPositionInches, double newYPositionInches) {
        setCurrentPositionInches(newXPositionInches, newYPositionInches, getCurrentHeading());
    }

    public void setCurrentPositionInches(double newXPositionInches, double newYPositionInches, WrappedRotation2d heading) {
        totalDistanceXinches.set(newXPositionInches);
        totalDistanceYinches.set(newYPositionInches);
        setCurrentHeading(heading.getDegrees());

        Pose2d metricPositionToResetTo = new Pose2d(
                newXPositionInches / PoseSubsystem.INCHES_IN_A_METER,
                newYPositionInches / PoseSubsystem.INCHES_IN_A_METER,
                this.getCurrentHeading());

        swerveOdometry.resetPosition(
                heading,
                currentModulePositionsSupplier.get(),
                metricPositionToResetTo);
    }

    public void setCurrentPoseInMeters(Pose2d newPoseInMeters) {
        setCurrentPositionInches(
                newPoseInMeters.getTranslation().getX() * PoseSubsystem.INCHES_IN_A_METER,
                newPoseInMeters.getTranslation().getY() * PoseSubsystem.INCHES_IN_A_METER,
                WrappedRotation2d.fromRotation2d(newPoseInMeters.getRotation())
        );
    }

    public FieldPose getCurrentFieldPoseInInches() {
        return new FieldPose(getTravelVectorInInches(), getCurrentHeading());
    }

    private XYPair getTravelVectorInInches() {
        return new XYPair(
                currentPoseWithGyroHeadingMetric.getX() * PoseSubsystem.INCHES_IN_A_METER,
                currentPoseWithGyroHeadingMetric.getY() * PoseSubsystem.INCHES_IN_A_METER);
    }

    public Pose2d getCurrentPoseWithVisionHeadingMetric() {
        return new Pose2d(
                currentPoseWithVisionHeadingMetric.getTranslation(),
                currentPoseWithVisionHeadingMetric.getRotation());
    }

    public Pose2d getCurrentPoseWithGyroHeadingMetric() {
        return new Pose2d(
                currentPoseWithGyroHeadingMetric.getTranslation(),
                currentPoseWithGyroHeadingMetric.getRotation());
    }


}
