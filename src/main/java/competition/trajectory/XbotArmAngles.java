package competition.trajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import xbot.common.math.XYPair;

public class XbotArmAngles implements ProvidesInterpolationData{

    private Translation2d angles;
    private double secondsForSegment;

    public XbotArmAngles(Translation2d angles, double secondsForSegment) {
        this.angles = angles;
        this.secondsForSegment = secondsForSegment;
    }

    @Override
    public double getSecondsForSegment() {
        return secondsForSegment;
    }

    // For a single angle, we'll just interpolate on X.
    @Override
    public Translation2d getTranslation2d() {
        return angles;
    }

    @Override
    public Rotation2d getRotation2d() {
        return null; // Not relevant for arm
    }
}
