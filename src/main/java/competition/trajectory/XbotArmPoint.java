package competition.trajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class XbotArmPoint implements ProvidesInterpolationData {

    private Translation2d point;

    private double secondsForSegment;

    public XbotArmPoint(Translation2d point, double secondsForSegment) {
        this.point = point;
        this.secondsForSegment = secondsForSegment;
    }

    @Override
    public Translation2d getTranslation2d() {
        return point;
    }

    @Override
    public double getSecondsForSegment() {
        return secondsForSegment;
    }

    @Override
    public Rotation2d getRotation2d() {
        return null; // Not relevant for arm
    }
}
