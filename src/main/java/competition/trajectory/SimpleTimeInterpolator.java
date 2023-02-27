package competition.trajectory;

import edu.wpi.first.math.geometry.Translation2d;
import org.apache.log4j.LogManager;
import org.apache.log4j.Logger;
import xbot.common.controls.sensors.XTimer;

import java.util.List;
public class SimpleTimeInterpolator {

    double accumulatedProductiveSeconds;
    double previousTimestamp;
    ProvidesInterpolationData baseline;
    int index;
    double maximumDistanceFromChasePointInInches = 12;

    private List<? extends ProvidesInterpolationData> keyPoints;

    Logger log = LogManager.getLogger(SimpleTimeInterpolator.class);

    public class InterpolationResult {
        public Translation2d chasePoint;
        public boolean isOnFinalPoint;

        public InterpolationResult(Translation2d chasePoint, boolean isOnFinalPoint) {
            this.chasePoint = chasePoint;
            this.isOnFinalPoint = isOnFinalPoint;
        }
    }

    public SimpleTimeInterpolator() {}

    public void setKeyPoints(List<? extends ProvidesInterpolationData > keyPoints) {
        this.keyPoints = keyPoints;
    }

    public void setMaximumDistanceFromChasePointInInches(double maximumDistanceFromChasePointInInches) {
        this.maximumDistanceFromChasePointInInches = maximumDistanceFromChasePointInInches;
    }

    public void initialize(ProvidesInterpolationData baseline) {
        this.baseline = baseline;
        accumulatedProductiveSeconds = 0;
        previousTimestamp = XTimer.getFPGATimestamp();
        index = 0;
    }

    public InterpolationResult calculateTarget(Translation2d currentLocation) {
        double secondsSinceLastExecute = XTimer.getFPGATimestamp() - previousTimestamp;
        accumulatedProductiveSeconds += secondsSinceLastExecute;

        // If we somehow have no points to visit, don't do anything.
        if (keyPoints.size() == 0) {
            log.warn("No key points to visit!");
            return new InterpolationResult(currentLocation, true);
        }

        var targetKeyPoint = keyPoints.get(index);

        if (targetKeyPoint.getSecondsForSegment() <= 0) {
            log.warn("Cannot have a keypoint with a time of 0 or less!");
            return new InterpolationResult(currentLocation, true);
        }

        // First, assume we are just going to our target. (This is what all trajectories will eventually
        // settle to - all this interpolation is for intermediate points.)
        Translation2d chasePoint = targetKeyPoint.getTranslation2d();

        // Now, try to find a better point via linear interpolation.
        double lerpFraction = (accumulatedProductiveSeconds) / targetKeyPoint.getSecondsForSegment();

        // If the fraction is above 1, it's time to set a new baseline point and start LERPing on the next
        // one.
        if (lerpFraction >= 1 && index < keyPoints.size()-1) {
            // What was our target now becomes our baseline.
            baseline = targetKeyPoint;
            accumulatedProductiveSeconds = 0;

            index++;
            // And set our new target to the next element of the list
            targetKeyPoint = keyPoints.get(index);
        }

        // Most of the time, the fraction will be less than one.
        // In that case, we want to interpolate between the baseline and the target.
        if (lerpFraction < 1) {
            chasePoint = baseline.getTranslation2d().interpolate(
                    targetKeyPoint.getTranslation2d(), lerpFraction);
        }

        // But if that chase point is "too far ahead", we need to freeze the chasePoint
        // until the robot has a chance to catch up.
        if (currentLocation.getDistance(chasePoint) > maximumDistanceFromChasePointInInches) {
            // This effectively "rewinds time" for the next loop.
            accumulatedProductiveSeconds -= secondsSinceLastExecute;
        }

        boolean targetingFinalPoint = index == keyPoints.size()-1 && lerpFraction >= 1;
        return new InterpolationResult(chasePoint, targetingFinalPoint);
    }


}
