package competition.subsystems.arm.commands;

import competition.subsystems.arm.ArmPositionState;
import competition.subsystems.arm.UnifiedArmSubsystem;
import xbot.common.command.BaseSetpointCommand;
import xbot.common.math.XYPair;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;

/**
 * Moves the end-effector by some distance in X-Z space
 */
public class ControlEndEffectorPositionCommand extends BaseSetpointCommand {

    private final UnifiedArmSubsystem arm;
    private final DoubleProperty positionChangePerStep;

    private XYPair movementVector = new XYPair(0, 0);
    private XYPair position;

    private boolean exceededLimits;

    @Inject
    public ControlEndEffectorPositionCommand(PropertyFactory pf, UnifiedArmSubsystem arm) {
        super(arm);
        this.arm = arm;
        pf.setPrefix(this);
        this.positionChangePerStep = pf.createPersistentProperty("Position change per step", 0.5);
        this.exceededLimits = false;
    }

    public void setDirection(XYPair vector) {
        movementVector = vector.clone().scale(this.positionChangePerStep.get());
    }

    @Override
    public void initialize() {
        position = arm.getCurrentXZCoordinates();
        arm.setDisableBrake(true);
    }

    @Override
    public void execute() {
        position = arm.constrainXZPosition(position.add(movementVector));

        ArmPositionState newTargets = arm.solver.solveArmJointPositions(position, arm.getCurrentValue(), false);
        if (arm.lowerArm.isAngleWithinLimits(newTargets.getLowerJointRotation().getDegrees())
                && arm.upperArm.isAngleWithinLimits(newTargets.getUpperJointRotation().getDegrees())) {
            arm.setTargetValue(new XYPair(newTargets.getLowerJointRotation().getDegrees(), newTargets.getUpperJointRotation().getDegrees()));
        } else {
            arm.setTargetValue(arm.getCurrentValue());
            exceededLimits = true;
        }
    }

    @Override
    public boolean isFinished() {
        return exceededLimits;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        arm.setDisableBrake(false);
    }
}
