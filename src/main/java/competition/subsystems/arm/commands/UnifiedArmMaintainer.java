package competition.subsystems.arm.commands;

import competition.operator_interface.OperatorInterface;
import competition.subsystems.arm.UnifiedArmSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import xbot.common.command.BaseMaintainerCommand;
import xbot.common.logic.HumanVsMachineDecider;
import xbot.common.math.MathUtils;
import xbot.common.math.XYPair;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;

public class UnifiedArmMaintainer extends BaseMaintainerCommand<XYPair> {

    UnifiedArmSubsystem unifiedArm;
    OperatorInterface oi;
    @Inject
    public UnifiedArmMaintainer(
            UnifiedArmSubsystem subsystemToMaintain,
            PropertyFactory pf,
            OperatorInterface oi,
            HumanVsMachineDecider.HumanVsMachineDeciderFactory hvmFactory) {
        super(subsystemToMaintain, pf, hvmFactory, 0.001, 0.001);
        this.unifiedArm = subsystemToMaintain;
        this.oi = oi;
    }

    @Override
    public void initialize() {
        log.info("Initializing");
    }

    @Override
    protected void coastAction() {
        unifiedArm.setPower(new XYPair(0,0));
    }

    @Override
    protected void calibratedMachineControlAction() {
        XYPair target = unifiedArm.getTargetValue();
        // Find out what angles the arms need to be at in order to achieve the goal.


        // Eventually do this. For now just do direct angle setting.
        /*
        var desiredArmAngles = unifiedArm.solver.solveArmJointPositions(target.x, target.y);
        // Ask the subsystem to move the arms to those angles.
        unifiedArm.setArmsToAngles(desiredArmAngles.getLowerJointRotation(), desiredArmAngles.getUpperJointRotation());

         */
        unifiedArm.setArmsToAngles(Rotation2d.fromDegrees(target.x), Rotation2d.fromDegrees(target.y));
    }

    @Override
    protected double getErrorMagnitude() {
        XYPair current = unifiedArm.getCurrentValue();
        XYPair target = unifiedArm.getTargetValue();

        double lowerArmError = Math.abs(current.x - target.x);
        double upperArmError = Math.abs(current.y - target.y);

        return lowerArmError + upperArmError;
    }

    @Override
    protected XYPair getHumanInput() {
        double lowerArmPower = MathUtils.deadband(
                oi.operatorGamepad.getLeftVector().y,
                oi.getOperatorGamepadTypicalDeadband(),
                (a)-> MathUtils.exponentAndRetainSign(a,2));
        double upperArmPower = MathUtils.deadband(
                oi.operatorGamepad.getRightVector().y,
                oi.getOperatorGamepadTypicalDeadband(),
                (a)-> MathUtils.exponentAndRetainSign(a, 2));

        return new XYPair(
                lowerArmPower,upperArmPower
        );
    }

    @Override
    protected double getHumanInputMagnitude() {
        return getHumanInput().getMagnitude();
    }

    /*
    @Override
    protected void humanControlAction() {
        unifiedArm.engageBrake(false);
        super.humanControlAction();
    }
    */
}
