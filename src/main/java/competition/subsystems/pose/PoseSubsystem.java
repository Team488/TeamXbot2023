package competition.subsystems.pose;

import javax.inject.Inject;
import javax.inject.Singleton;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.vision.VisionSubsystem;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import xbot.common.controls.sensors.XTimer;
import xbot.common.controls.sensors.XGyro.XGyroFactory;
import xbot.common.math.FieldPose;
import xbot.common.math.WrappedRotation2d;
import xbot.common.math.XYPair;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.pose.BasePoseSubsystem;

@Singleton
public class PoseSubsystem extends BasePoseSubsystem {

    private final DriveSubsystem drive;
    final SwerveDrivePoseEstimator swerveOdometry;
    private final VisionSubsystem vision;

    @Inject
    public PoseSubsystem(XGyroFactory gyroFactory, PropertyFactory propManager, DriveSubsystem drive, VisionSubsystem vision) {
        super(gyroFactory, propManager);
        this.drive = drive;
        this.vision = vision;

        /* Remember: WPILib uses a different coordinate convention than our legacy code. Theirs:
        //   0,+y. 90 degrees
        //       ----------------------------
        //       |                          |
        // Driver|                          |
        //       |                          |
        //       |                          |
        //       ----------------------------
        //     0,0                        +x,0. 0 degrees
        //
        // Our  code:
        //     0,0                        +y,90 degrees
        //       ----------------------------
        //       |                          |
        // Driver|                          |
        //       |                          |
        //       |                          |
        //       ----------------------------
        //     x,0. 0 degrees
        //
        // However, this isn't really a big deal - since both systems respect the "Right hand rule" (e.g. the Y axis is 90 degrees CCW to the X axis)
        // we can just ignore the difference. Any tool that looks at our position will be confused, since we will appear to be driving outside the field,
        // but it will work just fine for our own code.
        // That being said, at some point we should probably switch to the WPILib convention, since there are a number of path-planning tools and other
         utilities that expect the same conventions. */

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
        // In principle the input/output here is unitless, but we're using meters for consistency.

        // Try to get some vision sauce in there
        XYPair aprilCoords = vision.getAprilCoodinates();
        if (aprilCoords != null) {
            Pose2d aprilPos = new Pose2d(-aprilCoords.y, aprilCoords.x, new Rotation2d(0));
            swerveOdometry.addVisionMeasurement(aprilPos, XTimer.getFPGATimestamp() - 0.030);
        }

        Pose2d updatedPosition = swerveOdometry.update(
            this.getCurrentHeading(),
            getSwerveModulePositions()
        );       

        // Convert back to inches
        totalDistanceX.set(updatedPosition.getY() * PoseSubsystem.INCHES_IN_A_METER);
        totalDistanceY.set(-updatedPosition.getX() * PoseSubsystem.INCHES_IN_A_METER);
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

    private SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[] {
            drive.getFrontLeftSwerveModuleSubsystem().getcurrentPosition(),
            drive.getFrontRightSwerveModuleSubsystem().getcurrentPosition(),
            drive.getRearLeftSwerveModuleSubsystem().getcurrentPosition(),
            drive.getRearRightSwerveModuleSubsystem().getcurrentPosition()
        };
    }

}