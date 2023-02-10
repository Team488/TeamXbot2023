package competition.subsystems.arm.commands;

import competition.operator_interface.OperatorInterface;
import competition.subsystems.arm.UnifiedArmSubsystem;
import competition.subsystems.drive.swerve.SwerveSteeringSubsystem;
import edu.wpi.first.math.MathUtil;
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
            OperatorInterface oi,
            PropertyFactory pf,
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
        var desiredArmAngles = unifiedArm.solver.solveArmJointPositions(target.x, target.y);
        // Ask the subsystem to move the arms to those angles.
        unifiedArm.setArmsToAngles(desiredArmAngles.getLowerJointRotation(), desiredArmAngles.getUpperJointRotation());
    }

    @Override
    protected double getErrorMagnitude() {
        return 0;
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
}
