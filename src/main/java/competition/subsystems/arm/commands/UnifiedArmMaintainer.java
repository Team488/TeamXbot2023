package competition.subsystems.arm.commands;

import competition.operator_interface.OperatorInterface;
import competition.subsystems.arm.UnifiedArmSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import xbot.common.command.BaseMaintainerCommand;
import xbot.common.logic.HumanVsMachineDecider;
import xbot.common.math.MathUtils;
import xbot.common.math.XYPair;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;

public class UnifiedArmMaintainer extends BaseMaintainerCommand<XYPair> {

    UnifiedArmSubsystem unifiedArm;
    OperatorInterface oi;
    private final DoubleProperty lowerArmErrorThresholdToEngageBrake;
    private final DoubleProperty lowerArmErrorThresholdToDisengageBrake;
    @Inject
    public UnifiedArmMaintainer(
            UnifiedArmSubsystem subsystemToMaintain,
            PropertyFactory pf,
            OperatorInterface oi,
            HumanVsMachineDecider.HumanVsMachineDeciderFactory hvmFactory) {
        super(subsystemToMaintain, pf, hvmFactory, 0.001, 0.001);
        this.unifiedArm = subsystemToMaintain;
        this.oi = oi;
        pf.setPrefix(this);
        lowerArmErrorThresholdToEngageBrake = pf.createPersistentProperty("LowerArmErrorThresholdToEngageBrake",2.0);
        lowerArmErrorThresholdToDisengageBrake = pf.createPersistentProperty("LowerArmErrorThresholdToDisengageBrake",4.0);
    }

    @Override
    public void initialize() {
        log.info("Initializing");
    }

    @Override
    protected void humanControlAction() {
        // If the human is trying to move the arm, we should disable the brake.
        if (getHumanInputMagnitude() > decider.getDeadband())
        {
            unifiedArm.setBrake(false);
        } else {
            // Otherwise, we should engage the brake. Normally this wouldn't be possible (since
            // humanControlAction is only called when the decider is above the deadband) but it
            // could happen if the robot is in an "uncalibrated" mode.
            unifiedArm.setBrake(true);
        }
        super.humanControlAction();
    }

    @Override
    protected void coastAction() {
        unifiedArm.setBrake(true);
        unifiedArm.setPower(new XYPair(0,0));
    }

    @Override
    protected void initializeMachineControlAction() {
        unifiedArm.setBrake(true);
        super.initializeMachineControlAction();
    }

    @Override
    protected void calibratedMachineControlAction() {
        XYPair target = unifiedArm.getTargetValue();
        // Find out what angles the arms need to be at in order to achieve the goal.

        // If the lower arm is at the target angle, we should engage the brake.
        // However, this is vulnerable to cases where the PID isn't well-calibrated and the arm is stuck just
        // outside the critical boundary. For those cases, we could consider some time-based system.
        changeBrakeStateBasedOnError();

        // Eventually do this. For now just do direct angle setting.
        /*
        var desiredArmAngles = unifiedArm.solver.solveArmJointPositions(target.x, target.y);
        // Ask the subsystem to move the arms to those angles.
        unifiedArm.setArmsToAngles(desiredArmAngles.getLowerJointRotation(), desiredArmAngles.getUpperJointRotation());

         */
        unifiedArm.setArmsToAngles(Rotation2d.fromDegrees(target.x), Rotation2d.fromDegrees(target.y));
    }

    private void changeBrakeStateBasedOnError() {
        double lowerArmError = Math.abs(unifiedArm.getCurrentValue().x - unifiedArm.getTargetValue().x);

        if (lowerArmError < lowerArmErrorThresholdToEngageBrake.get()) {
            unifiedArm.setBrake(true);
        }

        if (lowerArmError > lowerArmErrorThresholdToDisengageBrake.get()) {
            unifiedArm.setBrake(false);
        }
    }

    @Override
    protected double getErrorMagnitude() {
        XYPair current = unifiedArm.getCurrentValue();
        XYPair target = unifiedArm.getTargetValue();

        double lowerArmError = 0;
        // If the brakes are engaged, we consider the lower arm error to be zero.
        // If the arm is free, then we read its error as normal.
        if (!unifiedArm.areBrakesEngaged()) {
            lowerArmError = Math.abs(current.x - target.x);
        }
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

    public double getLowerArmErrorThresholdToEngageBrake() {
        return lowerArmErrorThresholdToEngageBrake.get();
    }

    public double getLowerArmErrorThresholdToDisengageBrake() {
        return lowerArmErrorThresholdToDisengageBrake.get();
    }
}
