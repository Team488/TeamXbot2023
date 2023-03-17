package competition.subsystems.arm.commands;

import competition.subsystems.arm.ArmPositionState;
import competition.subsystems.arm.UnifiedArmSubsystem;
import competition.trajectory.SimpleTimeInterpolator;
import competition.trajectory.XbotArmPoint;
import edu.wpi.first.math.geometry.Translation2d;
import xbot.common.command.BaseSetpointCommand;
import xbot.common.math.XYPair;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class SimpleXZRouterCommand extends BaseSetpointCommand {

    UnifiedArmSubsystem arms;
    SimpleTimeInterpolator interpolator;
    private Supplier<XbotArmPoint> keyPointsProvider;
    SimpleTimeInterpolator.InterpolationResult lastResult;
    private ArrayList<XbotArmPoint> pointsToInterpolate;
    private UnifiedArmSubsystem.KeyArmPosition targetArmPosition;
    private UnifiedArmSubsystem.RobotFacing targetRobotFacing;
    private double defaultSegmentTime = 0.5;
    private final DoubleProperty defaultSegmentVelocity;

    @Inject
    public SimpleXZRouterCommand(UnifiedArmSubsystem arms, PropertyFactory pf) {
        super(arms);
        this.arms = arms;
        this.interpolator = new SimpleTimeInterpolator();
        pf.setPrefix(this);
        this.defaultSegmentVelocity = pf.createPersistentProperty("DefaultSegmentVelocity", 10);
    }

    public void setKeyPointProvider(Supplier<XbotArmPoint> keyPointsProvider) {
        this.keyPointsProvider = keyPointsProvider;
    }

    public void setKeyPoint(XbotArmPoint keyPoint) {
        setKeyPointProvider(() -> keyPoint);
    }

    public void setKeyPointFromDirectAngles(XYPair keyPoint) {
        setKeyPointProvider(
                () -> new XbotArmPoint(
                        arms.convertOldArmAnglesToXZPositions(keyPoint),
                        defaultSegmentTime)
        );
    }

    public void setKeyPointFromKeyArmPosition(
            UnifiedArmSubsystem.KeyArmPosition keyArmPosition,
            UnifiedArmSubsystem.RobotFacing facing) {
        setKeyPointProvider(() -> new XbotArmPoint(arms.getKeyArmXZ(keyArmPosition, facing), defaultSegmentTime));
    }

    public void setKeyPointFromKeyArmPositionProvider(
            Supplier<UnifiedArmSubsystem.KeyArmPosition> keyArmPositionProvider,
            Supplier<UnifiedArmSubsystem.RobotFacing> facingProvider) {
        setKeyPointProvider(() -> new XbotArmPoint(arms.getKeyArmXZ(keyArmPositionProvider.get(), facingProvider.get()), defaultSegmentTime));
    }

    @Override
    public void initialize() {
        log.info("Intializing");

        // New logic: get to any point in two steps.
        // If the target point is higher than us, we first go to a transition point wih the same X value of our current
        // position and a Z value of our target, then we go to the target point. (up, then over).
        // If the target position is below us, we first go to a transition point with the same Z value of our current
        // position and a X value of our target, then we go to the target point. (over, then down).

        var targetTranslation = keyPointsProvider.get().getTranslation2d();
        var currentArmCoordinates = arms.getCurrentXZCoordinatesAsTranslation2d();
        ArrayList<XbotArmPoint> waypoints = new ArrayList<>();

        // Check if we are above or below the target
        Translation2d transitionPoint2d = null;
        if (targetTranslation.getY() > currentArmCoordinates.getY()) {
            // The target is above us, so we need to go up, then over.
            transitionPoint2d = new Translation2d(currentArmCoordinates.getX(), targetTranslation.getY());
        } else {
            // The target is below us, so we need to go over, then down.
            transitionPoint2d = new Translation2d(targetTranslation.getX(), currentArmCoordinates.getY());
        }

        // Calculate the segment time using the desired velocity. We can get the distance between our current position
        // and our target position, and then divide that by the desired velocity to get the time.
        double distanceToTransitionPoint = currentArmCoordinates.getDistance(transitionPoint2d);
        var velocity = defaultSegmentVelocity.get();
        double firstSegmentTime = 1.0;
        if (velocity != 0) {
            firstSegmentTime = distanceToTransitionPoint / velocity;
        }
        waypoints.add(new XbotArmPoint(transitionPoint2d, firstSegmentTime));

        // Similarly, for the second segment, we can get the distance between the transition point and the target point.
        double distanceToTarget = transitionPoint2d.getDistance(targetTranslation);
        double secondSegmentTime = 1.0;
        if (velocity != 0) {
            secondSegmentTime = distanceToTarget / velocity;
        }

        waypoints.add(new XbotArmPoint(targetTranslation, secondSegmentTime));
        
        pointsToInterpolate = waypoints;
        interpolator.setKeyPoints(waypoints);
        interpolator.initialize(new XbotArmPoint(currentArmCoordinates, defaultSegmentTime));
        arms.setDisableBrake(true);
    }

    public List<XbotArmPoint> getPointsToInterpolate() {
        return pointsToInterpolate;
    }

    @Override
    public void execute() {

        var currentAngles = arms.getCurrentValue();

        lastResult = interpolator.calculateTarget(arms.getCurrentXZCoordinatesAsTranslation2d());
        var targetPosition = new XYPair(lastResult.chasePoint.getX(), lastResult.chasePoint.getY());
        var constrainedPosition = arms.constrainXZPosition(targetPosition);

        ArmPositionState newTargets = arms.solver.solveArmJointPositions(constrainedPosition, arms.getCurrentValue(), true);

        arms.setGhostArm(new Translation2d(
                newTargets.getLowerJointRotation().getDegrees(),
                newTargets.getUpperJointRotation().getDegrees()));

        if (newTargets.isSolveable()) {
            arms.setTargetValue(new XYPair(
                    newTargets.getLowerJointRotation().getDegrees(),
                    newTargets.getUpperJointRotation().getDegrees()));
        } else {
            arms.setTargetValue(arms.getCurrentValue());
        }

        if (lastResult.isOnFinalPoint) {
            arms.setDisableBrake(false);
        }
    }

    @Override
    public boolean isFinished() {
        if (arms.isMaintainerAtGoal() && lastResult.isOnFinalPoint) {
            log.info("Finished");
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        arms.setDisableBrake(false);
    }
}
