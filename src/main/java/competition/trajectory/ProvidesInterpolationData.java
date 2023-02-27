package competition.trajectory;

import edu.wpi.first.math.geometry.Translation2d;

public interface ProvidesInterpolationData {
    public Translation2d getTranslation2d();

    public double getSecondsForSegment();
}
