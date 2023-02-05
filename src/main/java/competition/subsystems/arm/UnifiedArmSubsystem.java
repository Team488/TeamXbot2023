package competition.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import xbot.common.command.BaseSetpointSubsystem;
import xbot.common.logic.HumanVsMachineDecider;
import xbot.common.math.XYPair;

import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public class UnifiedArmSubsystem extends BaseSetpointSubsystem<XYPair> {

    LowerArmSegment lowerArm;
    UpperArmSegment upperArm;

    private XYPair targetPosition;
    public final ArmPositionSolver solver;

    private HumanVsMachineDecider humanVsMachineDecider;

    public enum KeyArmPosition {
        lowerGoal,
        midGoal,
        highGoal,
        fullyRetracted
    }

    public static XYPair fullyRetractedPosition = new XYPair(0, 0);
    public static XYPair lowerGoalPosition = new XYPair(1*12, 0);
    public static XYPair midGoalPosition = new XYPair(3*12, 2*12);
    public static XYPair highGoalPosition = new XYPair(4*12, 3*12);

    @Inject
    public UnifiedArmSubsystem(
            LowerArmSegment lowerArm,
            UpperArmSegment upperArm) {
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

    public XYPair getKeyArmPosition(KeyArmPosition keyArmPosition){
        switch (keyArmPosition){
            case lowerGoal:
                return lowerGoalPosition;
            case midGoal:
                return midGoalPosition;
            case highGoal:
                return highGoalPosition;
            case fullyRetracted:
                return fullyRetractedPosition;
            default:
                return new XYPair(0,0);
        }
    }

    /**
     * Sets the power of the lower and upper arm motors directly (no PID)
     * @param lowerPower power of the lower arm motor
     * @param upperPower power of the upper arm motor
     */
    public void setArmPowers(double lowerPower, double upperPower) {
        lowerArm.setPower(lowerPower);
        upperArm.setPower(upperPower);
    }



    @Override
    public XYPair getCurrentValue() {
        return solver.getPositionFromRadians(
            lowerArm.getArmPositionInRadians(),
            upperArm.getArmPositionInRadians());
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
        lowerArm.setPower(power.x);
        upperArm.setPower(power.y);
    }

    public void setArmsToAngles(Rotation2d lowerArmAngle, Rotation2d upperArmAngle) {
        lowerArm.setArmToAngle(lowerArmAngle);
        upperArm.setArmToAngle(upperArmAngle);
    }

    @Override
    public boolean isCalibrated() {
        return false;
    }
}
