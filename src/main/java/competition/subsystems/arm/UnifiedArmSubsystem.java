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
    public final ArmPositionSolver solver;

    private HumanVsMachineDecider humanVsMachineDecider;

    @Inject
    public UnifiedArmSubsystem(
            LowerArmSubsystem lowerArm,
            UpperArmSubsystem upperArm) {
        this.lowerArm = lowerArm;
        this.upperArm = upperArm;
        ArmPositionSolverConfiguration armConfig = new ArmPositionSolverConfiguration(
            38.0,
            32.0,
            Rotation2d.fromDegrees(15.0),
            Rotation2d.fromDegrees(165.0),
            Rotation2d.fromDegrees(-175),
            Rotation2d.fromDegrees(-5.0)
        );
        solver = new ArmPositionSolver(armConfig);
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

    @Override
    public XYPair getCurrentValue() {
        return solver.getPositionFromAngles(
            lowerArm.getAngle(),
            upperArm.getAngle()
        );
    }

    @Override
    public XYPair getTargetValue() {
        return targetPosition;
    }

    @Override
    public void setTargetValue(XYPair value) {
        this.targetPosition = value;
    }

    /**
     * Sets the power of the arm motors directly, mapping X to lower arm and Y to upper arm.
     * @param power X - lower arm power. Y - upper arm power.
     */
    @Override
    public void setPower(XYPair power) {
        lowerArm.setMotorPower(power.x);
        upperArm.setMotorPower(power.y);
    }

    public void setArmsToAngles() {
        upperArm.set
    }

    @Override
    public boolean isCalibrated() {
        return false;
    }
}
