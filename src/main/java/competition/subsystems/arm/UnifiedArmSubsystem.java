package competition.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import xbot.common.command.BaseSetpointSubsystem;
import xbot.common.command.BaseSubsystem;
import xbot.common.logic.HumanVsMachineDecider;
import xbot.common.logic.HumanVsMachineDecider_Factory;
import xbot.common.math.XYPair;

import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public class UnifiedArmSubsystem extends BaseSetpointSubsystem<XYPair> {

    LowerArmSubsystem lowerArm;
    UpperArmSubsystem upperArm;

    private XYPair targetPosition;
    private ArmPositionSolver solver;

    private HumanVsMachineDecider humanVsMachineDecider;

    @Inject
    public UnifiedArmSubsystem(
            LowerArmSubsystem lowerArm,
            UpperArmSubsystem upperArm,
            HumanVsMachineDecider.HumanVsMachineDeciderFactory hvmFactory) {
        this.lowerArm = lowerArm;
        this.upperArm = upperArm;
        this.humanVsMachineDecider = hvmFactory.create(this.getPrefix());
        ArmPositionSolverConfiguration armConfig = new ArmPositionSolverConfiguration(
            38.0,
            32.0,
            Rotation2d.fromDegrees(15.0),
            Rotation2d.fromDegrees(165.0),
            Rotation2d.fromDegrees(-175),
            Rotation2d.fromDegrees(-5.0)
        );
        ArmPositionSolver solver = new ArmPositionSolver(armConfig);
    }

    /**
     * Sets the power of the lower and upper arm motors directly (no PID)
     * @param lowerPower power of the lower arm motor
     * @param upperPower power of the upper arm motor
     */
    public void setArmPowers(double lowerPower, double upperPower) {
        lowerArm.setMotorPower(lowerPower);
        upperArm.setMotorPower(upperPower);
    }

    /**
     * Sets the desired ** (X,Z) ** position of the arm. This is the position of the end effector.
     * @param targetPosition desired (X,Z) position of the end effector.
     */
    public void goToTargetPosition(XYPair targetPosition) {
        this.targetPosition = targetPosition;
    }

    public XYPair getCurrentPosition() {
        return new XYPair();
        //return solver.getPositionFromAngles(
        //    lowerArm.getCurrentAngle(),
        //    upperArm.getCurrentAngle()
        //);
    }

    @Override
    public XYPair getCurrentValue() {
        return null;
    }

    @Override
    public XYPair getTargetValue() {
        return null;
    }

    @Override
    public void setTargetValue(XYPair value) {

    }

    @Override
    public void setPower(XYPair power) {

    }

    @Override
    public boolean isCalibrated() {
        return false;
    }
}
