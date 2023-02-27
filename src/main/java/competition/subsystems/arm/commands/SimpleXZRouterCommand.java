package competition.subsystems.arm.commands;

import competition.subsystems.arm.ArmPositionState;
import competition.subsystems.arm.UnifiedArmSubsystem;
import competition.subsystems.drive.commands.XbotSwervePoint;
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
        pointsToConsider.add(keyPointsProvider.get());




        keyPoints = keyPointsProvider.get();
        var initialBaseline = arms.getCurrentXZCoordinatesAsTranslation2d();

        interpolator.setKeyPoints(keyPoints);
        interpolator.initialize(new XbotArmPoint(initialBaseline, 0));
        arms.setDisableBrake(true);
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
