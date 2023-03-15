package competition.subsystems.arm.commands;

import competition.subsystems.arm.UnifiedArmSubsystem;
import competition.trajectory.SimpleTimeInterpolator;
import competition.trajectory.XbotArmAngles;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import xbot.common.command.BaseSetpointCommand;

import javax.inject.Inject;
import java.util.ArrayList;

public class InterpolatingSafeArmRouterCommand extends BaseSetpointCommand {
    private final UnifiedArmSubsystem arms;
    private UnifiedArmSubsystem.KeyArmPosition targetArmPosition;
    private UnifiedArmSubsystem.RobotFacing targetRobotFacing;

    private final SimpleTimeInterpolator armAngleInterpolator;
    SimpleTimeInterpolator.InterpolationResult lastResult;

    private static Logger log = LogManager.getLogger(SimpleSafeArmRouterCommand.class);

    @Inject
    public InterpolatingSafeArmRouterCommand(UnifiedArmSubsystem arms) {
        super(arms);
        this.arms = arms;

        armAngleInterpolator = new SimpleTimeInterpolator();
    }

    public void setTarget(UnifiedArmSubsystem.KeyArmPosition targetArmPosition, UnifiedArmSubsystem.RobotFacing targetRobotFacing) {
        this.targetArmPosition = targetArmPosition;
        this.targetRobotFacing = targetRobotFacing;
    }

    @Override
    public void initialize() {
        log.info("Initializing");
        var currentAngles = arms.getCurrentValue();
        armAngleInterpolator.initialize(new XbotArmAngles(currentAngles.toTranslation2d(), 1.0));

        // New logic: get to any point in two steps.
        // If the target point is higher than us, we first go to a transition point wih the same X value of our current
        // position and a Z value of our target, then we go to the target point.

        // Now to build the key points
        ArrayList<XbotArmAngles> keyPoints = new ArrayList<>();

        log.info("Adding SafeExternalTransition for current side");
        keyPoints.add(new XbotArmAngles(arms.getKeyArmAngles(
                UnifiedArmSubsystem.KeyArmPosition.SafeExternalTransition,
                UnifiedArmSubsystem.RobotFacing.Forward).toTranslation2d(), 1.0));

        log.info("Adding target point");
        keyPoints.add(new XbotArmAngles(arms.getKeyArmAngles(
                targetArmPosition,
                UnifiedArmSubsystem.RobotFacing.Forward).toTranslation2d(), 1.0));

        // Since we just set the target, the maintainer hasn't had a chance to execute yet and evaluate to see if we
        // are at that. To avoid premature completion, we will force the subsystem to say the maintainer
        // is not yet at the goal, since we will be checking that value immediately in execute.
        arms.setMaintainerIsAtGoal(false);
        arms.setDisableBrake(true);
    }

    @Override
    public void execute() {
        var ghost = armAngleInterpolator.calculateTarget(arms.getCurrentValue().toTranslation2d());
        arms.setGhostArm(ghost.chasePoint);
        arms.setArmsToAngles(
                Rotation2d.fromDegrees(ghost.chasePoint.getX()),
                Rotation2d.fromDegrees(ghost.chasePoint.getY()));
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

