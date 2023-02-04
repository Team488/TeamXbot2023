package competition.subsystems.pose;

import javax.inject.Inject;
import javax.inject.Singleton;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.vision.VisionSubsystem;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;
import xbot.common.controls.sensors.XGyro.XGyroFactory;
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

@Singleton
public class PoseSubsystem extends BasePoseSubsystem {

    private final DriveSubsystem drive;
    final SwerveDrivePoseEstimator swerveOdometry;
    private final VisionSubsystem vision;
    private final Field2d fieldForDisplay;

    private boolean isPoseHealthy;
    private TimeStableValidator healthyPoseValidator = new TimeStableValidator(1);
    private final DoubleProperty suprisingVisionUpdateDistanceInMetersProp;
    private TimeStableValidator extremelyConfidentVisionValidator = new TimeStableValidator(10);
    private final DoubleProperty extremelyConfidentVisionDistanceUpdateInMetersProp;
    private boolean isVisionPoseExtremelyConfident;
    private final Latch useVisionToUpdateGyroLatch;


    @Inject
    public PoseSubsystem(XGyroFactory gyroFactory, PropertyFactory propManager, DriveSubsystem drive, VisionSubsystem vision) {
        super(gyroFactory, propManager);
        this.drive = drive;
        this.vision = vision;

        suprisingVisionUpdateDistanceInMetersProp = propManager.createPersistentProperty("SuprisingVisionUpdateDistanceInMeters", 0.5);
        extremelyConfidentVisionDistanceUpdateInMetersProp = propManager.createPersistentProperty("ExtremelyConfidentVisionDistanceUpdateInMeters", 0.01);

        // TODO: This is a hack to get the field visualization working. Eventually this is going to cause problems
        // once there are test cases that try and invoke the PoseSubsystem. Right now, the SmartDashboardCommandPutter
        // is the only class we have that manages direct calls to SmartDashboard. Maybe we should broaden it to a more
        // generic "XSmartDashboard" class for scenarios like these?
        fieldForDisplay = new Field2d();
        SmartDashboard.putData("Field", fieldForDisplay);

        // In the DriveSubsystem, the swerve modules were initialized in this order:
        // FrontLeft, FrontRight, RearLeft, RearRight.
        // When initializing SwerveDriveOdometry, we need to use the same order.

        swerveOdometry = new SwerveDrivePoseEstimator(
            drive.getSwerveDriveKinematics(), 
            getCurrentHeading(), 
            new SwerveModulePosition[] {
                drive.getFrontLeftSwerveModuleSubsystem().getcurrentPosition(),
                drive.getFrontRightSwerveModuleSubsystem().getcurrentPosition(),
                drive.getRearLeftSwerveModuleSubsystem().getcurrentPosition(),
                drive.getRearRightSwerveModuleSubsystem().getcurrentPosition()
            },
            new Pose2d());

        useVisionToUpdateGyroLatch = new Latch(false, Latch.EdgeType.RisingEdge, edge -> {
           if (edge== Latch.EdgeType.RisingEdge) {
               log.info("Vision has been so confident for so long that we are force-updating our overall pose.");
               this.setCurrentPoseInMeters(getVisionAssistedPositionInMeters());
           }
        });
    }


    /**
     * This is a legacy method for tank drive robots, and does not apply to swerve. We should look at
     * updating the base class to remove/replace this method.
     */
    @Override
    protected double getLeftDriveDistance() {
        return drive.getLeftTotalDistance();
    }

    /**
     * This is a legacy method for tank drive robots, and does not apply to swerve. We should look at
     * updating the base class to remove/replace this method.
     */
    @Override
    protected double getRightDriveDistance() {
        return drive.getRightTotalDistance();
    }

    @Override
    protected void updateOdometry() {
        // The swerve modules return units in meters, which is what the swerve odometry expects.
        // In principle the input/output here is unitless, but we're using meters internally for any calculations
        // while still presenting inches externally to dashboards.

        // Update the basic odometry (gyro, encoders)

        Pose2d updatedPosition = swerveOdometry.update(
                this.getCurrentHeading(),
                getSwerveModulePositions()
        );

        // As a prototype, consider any AprilTag seen to be at field coordinates 0,0. Use that information
        // to position the robot on the field.
        //improveOdometryUsingSimpleAprilTag();

        // As a better prototype, use PhotonLib to evaluate multiple AprilTags and get a field-accurate position.
        improveOdometryUsingPhotonLib(updatedPosition);

        // TODO: as an even better prototype, use a multi-target PNP solver (not yet available, but coming soon?)

        // Pull out the new estimated pose from odometry. Note that for now, we only pull out X and Y
        // and trust the gyro implicitly. Eventually, we should allow the gyro to be updated via vision
        // if we have a lot of confidence in the vision data.
        var estimatedPosition = swerveOdometry.getEstimatedPosition();

        // Convert back to inches
        totalDistanceX = estimatedPosition.getX() * PoseSubsystem.INCHES_IN_A_METER;
        totalDistanceY = estimatedPosition.getY() * PoseSubsystem.INCHES_IN_A_METER;
        fieldForDisplay.setRobotPose(estimatedPosition);
        Logger.getInstance().recordOutput("RobotEstimatedPose", estimatedPosition);
    }

    private void improveOdometryUsingSimpleAprilTag() {
        // Try to get some vision sauce in there
        // and feed it straight into the odometry, then do the shifting at the very end when we convert back to inches.
        XYPair aprilCoords = vision.getAprilCoordinates();
        if (aprilCoords != null) {
            Pose2d aprilPos = new Pose2d(aprilCoords.x, aprilCoords.y, getCurrentHeading());
            swerveOdometry.addVisionMeasurement(aprilPos, XTimer.getFPGATimestamp() - 0.030);
        }
    }

    private void improveOdometryUsingPhotonLib(Pose2d recentPosition) {
        var photonEstimatedPose = vision.getPhotonVisionEstimatedPose(recentPosition);

        if (photonEstimatedPose.isPresent()) {
            // Get the result data, which has both coordinates and timestamps
            var camPose = photonEstimatedPose.get();

            // Check for the distance delta between the old and new poses. If it's too large, reset
            // the healthy pose validator.
            double distance = recentPosition.getTranslation().getDistance(
                    camPose.estimatedPose.getTranslation().toTranslation2d());
            boolean isSurprisingDistance = (distance > suprisingVisionUpdateDistanceInMetersProp.get());
            isPoseHealthy = healthyPoseValidator.checkStable(!isSurprisingDistance);

            // If the distance delta is tiny, we're extremely confident in the vision data, and
            // could consider using it to update the gyro.
            boolean isExtremelyConfident = (distance < extremelyConfidentVisionDistanceUpdateInMetersProp.get());
            isVisionPoseExtremelyConfident = extremelyConfidentVisionValidator.checkStable(isExtremelyConfident);

            // In any case, update the odometry with the new pose from the camera.
            swerveOdometry.addVisionMeasurement(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
        } else
        {
            // Since we didn't get any vision updates, we assume the current pose is healthy.
            isPoseHealthy = healthyPoseValidator.checkStable(true);
            // But since we didn't get any vision updates, we can't be super-confident!
            isVisionPoseExtremelyConfident = extremelyConfidentVisionValidator.checkStable(false);
        }

        Logger.getInstance().recordOutput(this.getPrefix()+ "VisionPoseHealthy", isPoseHealthy);
        Logger.getInstance().recordOutput(this.getPrefix()+ "VisionPoseExtremelyConfident", isVisionPoseExtremelyConfident);
    }

    public boolean getIsPoseHealthy() {
        return isPoseHealthy;
    }

    public boolean getIsVisionPoseExtremelyConfident() {
        return isVisionPoseExtremelyConfident;
    }

    public Pose2d getVisionAssistedPositionInMeters() {
        return swerveOdometry.getEstimatedPosition();
    }

    public void setCurrentPosition(double newXPosition, double newYPosition, WrappedRotation2d heading) {
        super.setCurrentPosition(newXPosition, newYPosition);
        super.setCurrentHeading(heading.getDegrees());
        swerveOdometry.resetPosition(
            heading,
            getSwerveModulePositions(),
            new Pose2d(
                newXPosition / PoseSubsystem.INCHES_IN_A_METER, 
                newYPosition / PoseSubsystem.INCHES_IN_A_METER, 
                this.getCurrentHeading()));
    }

    @Override
    public void setCurrentPosition(double newXPosition, double newYPosition) {
        setCurrentPosition(newXPosition, newYPosition, this.getCurrentHeading());
    }

    public void setCurrentPose(FieldPose newPose) {
        setCurrentPosition(newPose.getPoint().x, newPose.getPoint().y, newPose.getHeading());
        this.setCurrentHeading(newPose.getHeading().getDegrees());
    }

    public void setCurrentPoseInMeters(Pose2d newPoseInMeters) {
        setCurrentPosition(
            newPoseInMeters.getTranslation().getX() * PoseSubsystem.INCHES_IN_A_METER,
            newPoseInMeters.getTranslation().getY() * PoseSubsystem.INCHES_IN_A_METER,
            WrappedRotation2d.fromRotation2d(newPoseInMeters.getRotation())
        );
    }


    private SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[] {
            drive.getFrontLeftSwerveModuleSubsystem().getcurrentPosition(),
            drive.getFrontRightSwerveModuleSubsystem().getcurrentPosition(),
            drive.getRearLeftSwerveModuleSubsystem().getcurrentPosition(),
            drive.getRearRightSwerveModuleSubsystem().getcurrentPosition()
        };
    }

}