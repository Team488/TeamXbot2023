package competition.subsystems.arm.commands;

import competition.operator_interface.OperatorInterface;
import competition.subsystems.arm.UnifiedArmSubsystem;
import competition.subsystems.drive.swerve.SwerveSteeringSubsystem;
import xbot.common.command.BaseMaintainerCommand;
import xbot.common.logic.HumanVsMachineDecider;
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
        var desiredArmAngles = unifiedArm.solver.solveArmJointPositions(target.x, target.y);
    }

    @Override
    protected double getErrorMagnitude() {
        return 0;
    }

    @Override
    protected XYPair getHumanInput() {
        return new XYPair(
                oi.operatorGamepad.getLeftVector().y,
                oi.operatorGamepad.getRightVector().y
        );
    }

    @Override
    protected double getHumanInputMagnitude() {
        return getHumanInput().getMagnitude();
    }
}
