package competition.subsystems.arm.commands;

import competition.subsystems.arm.ArmPositionState;
import competition.subsystems.arm.UnifiedArmSubsystem;
import competition.trajectory.SimpleTimeInterpolator;
import competition.trajectory.XbotArmPoint;
import edu.wpi.first.math.geometry.Translation2d;
import xbot.common.command.BaseSetpointCommand;
import xbot.common.math.XYPair;

import javax.inject.Inject;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class SimpleXZRouterCommand extends BaseSetpointCommand {

    UnifiedArmSubsystem arms;
    SimpleTimeInterpolator interpolator;
    private Supplier<XbotArmPoint> keyPointsProvider;
    private ArrayList<Translation2d> pointsToConsider = new ArrayList<>();
    SimpleTimeInterpolator.InterpolationResult lastResult;
    private ArrayList<XbotArmPoint> pointsToInterpolate;

    @Inject
    public SimpleXZRouterCommand(UnifiedArmSubsystem arms) {
        super(arms);
        this.arms = arms;
        this.interpolator = new SimpleTimeInterpolator();
    }

    public void setKeyPointsProvider(Supplier<XbotArmPoint> keyPointsProvider) {
        this.keyPointsProvider = keyPointsProvider;
    }

    public void setKeyPoint(XbotArmPoint keyPoint) {
        setKeyPointsProvider(() -> keyPoint);
    }

    public void setKeyPointFromDirectAngles(XYPair keyPoint) {
        setKeyPointsProvider(
                () -> new XbotArmPoint(
                        arms.convertOldArmAnglesToXZPositions(keyPoint),
                        0)
        );
    }

    @Override
    public void initialize() {
        // Now that we have XZ control, we can go to a much smarter "safe" point.
        // Basic idea - there are three "safe" points, and each should be easily reachable
        // from the others.
        // When we start, we compare our current position to the three safe points as well as our target point,
        // and go to the nearest one.
        // Each time we use a safe point, we remove it from consideration, and repeat the process.
        // Whenever the target point is the closest point, we go there and stop building a list of points.

        pointsToConsider = new ArrayList<>();
        pointsToConsider.add(UnifiedArmSubsystem.lowSafePosition);
        pointsToConsider.add(UnifiedArmSubsystem.midSafePosition);
        pointsToConsider.add(UnifiedArmSubsystem.highSafePosition);
        pointsToConsider.add(UnifiedArmSubsystem.mirrorXZPoints(UnifiedArmSubsystem.lowSafePosition));
        pointsToConsider.add(UnifiedArmSubsystem.mirrorXZPoints(UnifiedArmSubsystem.midSafePosition));
        pointsToConsider.add(UnifiedArmSubsystem.mirrorXZPoints(UnifiedArmSubsystem.highSafePosition));

        // The transition points need to be handled carefully, as this simple routing algorithm will
        // always choose the nearest point, and these are just a few inches from each other.
        // In the routine below, whenever we remove one from consideration, we need to remove the other.
        var specialPointForward = UnifiedArmSubsystem.specialMiddleTransitionPositionForward;
        var specialPointBackward = UnifiedArmSubsystem.mirrorXZPoints(specialPointForward);
        pointsToConsider.add(specialPointForward);
        pointsToConsider.add(specialPointBackward);

        var targetTranslation = keyPointsProvider.get().getTranslation2d();
        pointsToConsider.add(targetTranslation);

        var currentArmCoordinates = arms.getCurrentXZCoordinatesAsTranslation2d();
        var originPoint = currentArmCoordinates;

        // Now, find the next nearest waypoint/final point.
        Translation2d nearest = new Translation2d();

        ArrayList<XbotArmPoint> waypoints = new ArrayList<>();

        int escape = 0;
        while (nearest != targetTranslation) {
            nearest = originPoint.nearest(pointsToConsider);

            // As mentioned above, the special points need to be removed together.
            if (nearest == specialPointForward || nearest == specialPointBackward) {
                pointsToConsider.remove(specialPointBackward);
                pointsToConsider.remove(specialPointForward);
            } else {
                pointsToConsider.remove(nearest);
            }
            waypoints.add(new XbotArmPoint(nearest, 0.5));
            originPoint = nearest;
            escape++;
            // Just in case we do something very wrong, every while loop needs an escape hatch.
            // Otherwise, we run the risk of the robot getting stuck in some loop instead of driving around.
            if (escape > 20) {
                break;
            }
        }


        pointsToInterpolate = waypoints;
        interpolator.setKeyPoints(waypoints);
        interpolator.initialize(new XbotArmPoint(currentArmCoordinates, 0));
        arms.setDisableBrake(true);
    }

    public List<XbotArmPoint> getPointsToInterpolate() {
        return pointsToInterpolate;
    }

    @Override
    public void execute() {
        lastResult = interpolator.calculateTarget(arms.getCurrentXZCoordinatesAsTranslation2d());
        var targetPosition = new XYPair(lastResult.chasePoint.getX(), lastResult.chasePoint.getY());
        var constrainedPosition = arms.constrainXZPosition(targetPosition);

        ArmPositionState newTargets = arms.solver.solveArmJointPositions(constrainedPosition, arms.getCurrentValue());
        arms.setTargetValue(new XYPair(
                newTargets.getLowerJointRotation().getDegrees(),
                newTargets.getUpperJointRotation().getDegrees()));
    }

    @Override
    public boolean isFinished() {
        return arms.isMaintainerAtGoal() && lastResult.isOnFinalPoint;
    }

    @Override
    public void end(boolean interrupted) {
        arms.setDisableBrake(false);
    }
}
