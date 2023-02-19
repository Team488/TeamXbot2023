package competition.subsystems.drive.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

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
}
