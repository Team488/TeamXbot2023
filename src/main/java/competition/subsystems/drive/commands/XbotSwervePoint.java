package competition.subsystems.drive.commands;

import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import xbot.common.math.WrappedRotation2d;

import java.util.ArrayList;
import java.util.List;

public class XbotSwervePoint {

    public Pose2d keyPose;

    public double secondsToPoint;

    public XbotSwervePoint(Pose2d keyPose, double secondsToPoint) {
        this.keyPose = keyPose;
        this.secondsToPoint = secondsToPoint;
    }

    public XbotSwervePoint(double x, double y, double degrees, double secondsToPoint) {
        this.keyPose = new Pose2d(x, y, Rotation2d.fromDegrees(degrees));
        this.secondsToPoint = secondsToPoint;
    }

    public static Trajectory generateTrajectory(List<XbotSwervePoint> swervePoints) {
        ArrayList<Trajectory.State> wpiStates = new ArrayList<>();
        for (XbotSwervePoint point : swervePoints) {
            Trajectory.State state = new Trajectory.State();
            // Swerve points are in inches, but the trajectory is in meters.
            state.poseMeters = new Pose2d(
                    point.keyPose.getTranslation().getX() / PoseSubsystem.INCHES_IN_A_METER,
                    point.keyPose.getTranslation().getY() / PoseSubsystem.INCHES_IN_A_METER,
                    WrappedRotation2d.fromRotation2d(point.keyPose.getRotation())
            );
            state.velocityMetersPerSecond = 0;
            state.accelerationMetersPerSecondSq = 0;
            wpiStates.add(state);
        }
        return new Trajectory(wpiStates);
    }
}
