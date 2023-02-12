package competition.subsystems.arm;

import competition.electrical_contract.ElectricalContract;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import xbot.common.command.BaseSetpointSubsystem;
import xbot.common.controls.actuators.XSolenoid;
import xbot.common.math.XYPair;
import xbot.common.properties.BooleanProperty;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public class UnifiedArmSubsystem extends BaseSetpointSubsystem<XYPair> {

    LowerArmSegment lowerArm;
    UpperArmSegment upperArm;
    public XSolenoid lowerArmSolenoid;
    private XYPair targetPosition;
    public final ArmPositionSolver solver;
    private final DoubleProperty lowerArmTarget;
    private final DoubleProperty upperArmTarget;

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

    double testRangeRadians = 0.17453292519943295; // 10 degrees

    protected final BooleanProperty calibratedProp;

    protected final BooleanProperty areBrakesEngaged;

    @Inject
    public UnifiedArmSubsystem(
            LowerArmSegment lowerArm,
            UpperArmSegment upperArm,
            XSolenoid.XSolenoidFactory xSolenoidFactory,
            ElectricalContract eContract,
            PropertyFactory pf) {
        this.lowerArm = lowerArm;
        this.upperArm = upperArm;
        this.lowerArmSolenoid = xSolenoidFactory.create(eContract.getLowerArmBrakeSolenoid().channel);
        ArmPositionSolverConfiguration armConfig = new ArmPositionSolverConfiguration(
            38.0,
            32.0,
            Rotation2d.fromDegrees(15.0),
            Rotation2d.fromDegrees(165.0),
            Rotation2d.fromDegrees(-175),
            Rotation2d.fromDegrees(-5.0)
        );
        solver = new ArmPositionSolver(armConfig);
        pf.setPrefix(this);
        calibratedProp = pf.createEphemeralProperty("Calibrated", false);
        upperArmTarget = pf.createEphemeralProperty("UpperArmTarget", 0.0);
        lowerArmTarget = pf.createEphemeralProperty("LowerArmTarget", 0.0);
        areBrakesEngaged = pf.createEphemeralProperty("AreBrakesEngaged", false);
        targetPosition = getCurrentValue();

        areBrakesEngaged.set(true);
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

    @Override
    public XYPair getCurrentValue() {

        return new XYPair(
                lowerArm.getArmPositionFromAbsoluteEncoderInDegrees(),
                upperArm.getArmPositionFromAbsoluteEncoderInDegrees()
        );
        // Eventually do the smart thing. For now, just do angles.
        /*
        return solver.getPositionFromRadians(
            lowerArm.getArmPositionInRadians(),
            upperArm.getArmPositionInRadians());

         */
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
        if (areBrakesEngaged.get()) {
            lowerArm.setPower(0);
        } else {
            lowerArm.setPower(power.x);
        }
        upperArm.setPower(power.y);
    }

    public void setArmsToAngles(Rotation2d lowerArmAngle, Rotation2d upperArmAngle) {
        if (areBrakesEngaged.get()) {
            lowerArm.setPower(0);
        } else {
            lowerArm.setArmToAngle(lowerArmAngle);
        }
        upperArm.setArmToAngle(upperArmAngle);
    }

    @Override
    public boolean isCalibrated() {
        return calibratedProp.get();
    }

    public void calibrateAt(double lowerArmAngleInDegrees, double upperArmAngleInDegrees) {
        lowerArm.calibrateThisPositionAs(lowerArmAngleInDegrees);
        upperArm.calibrateThisPositionAs(upperArmAngleInDegrees);
    }

    public void calibrateAgainstAbsoluteEncoders() {
        lowerArm.calibrateThisPositionAs(lowerArm.getArmPositionFromAbsoluteEncoderInDegrees());
        upperArm.calibrateThisPositionAs(upperArm.getArmPositionFromAbsoluteEncoderInDegrees());
    }

    /**
     * Guessing at what the easiest position to calibrate at is.
     * Assuming lower arm straight up, upper arm straight down?
     */
    public void typicalCalibrate() {
        calibrateAt(90, -90);
    }

    public boolean isGivenPositionIllegal(XYPair position) {
        // If the end effector is too high, it's illegal.
        if (position.y > 0) {
            return true;
        }
        // If we are more than 48 inches forward or backward, it's illegal.
        // TODO: Update with end effector length in the calculation
        if (Math.abs(position.x) > 48) {
            return true;
        }
        return false;
    }

    public enum ArmRiskState {
        IncreaseAngleRisk,
        DecreaseAngleRisk,
        NoRisk
    }

    /**
     * Tests moving the lower and upper arm by 10 degrees to see if we're about to be in trouble.
     * This information should be used to temporarily restrict the arm's movement.
     * @return LowerArm risk, UpperArm risk
     */
    public Pair<ArmRiskState, ArmRiskState> checkArmRisk() {
        var lowerArmAngle = lowerArm.getArmPositionInRadians();
        var upperArmAngle = upperArm.getArmPositionInRadians();

        var lowerArmIncreaseAngleTestValue = lowerArmAngle + testRangeRadians;
        var lowerArmDecreaseAngleTestValue = lowerArmAngle - testRangeRadians;

        var upperArmIncreaseAngleTestValue = upperArmAngle + testRangeRadians;
        var upperArmDecreaseAngleTestValue = upperArmAngle - testRangeRadians;

        ArmRiskState lowerArmRiskState = ArmRiskState.NoRisk;
        ArmRiskState upperArmRiskState = ArmRiskState.NoRisk;

        if (isGivenPositionIllegal(solver.getPositionFromRadians(lowerArmIncreaseAngleTestValue, upperArmAngle))) {
            lowerArmRiskState = ArmRiskState.IncreaseAngleRisk;
        }
        if (isGivenPositionIllegal(solver.getPositionFromRadians(lowerArmDecreaseAngleTestValue, upperArmAngle))) {
            lowerArmRiskState = ArmRiskState.DecreaseAngleRisk;
        }

        if (isGivenPositionIllegal(solver.getPositionFromRadians(lowerArmAngle, upperArmIncreaseAngleTestValue))) {
            upperArmRiskState = ArmRiskState.IncreaseAngleRisk;
        }
        if (isGivenPositionIllegal(solver.getPositionFromRadians(lowerArmAngle, upperArmDecreaseAngleTestValue))) {
            upperArmRiskState = ArmRiskState.DecreaseAngleRisk;
        }

        return new Pair<>(lowerArmRiskState, upperArmRiskState);
    }

    //set brakes for lower arm
    public void engageBrake(boolean on){
        if(on){
            lowerArmSolenoid.setOn(false);
            areBrakesEngaged.set(true);
        } else {
            lowerArmSolenoid.setOn(true);
            areBrakesEngaged.set(false);
        }
    }

    public void setSoftLimits(boolean on) {
        lowerArm.setSoftLimit(on);
        upperArm.setSoftLimit(on);
    }

    @Override
    public void periodic() {
        lowerArmTarget.set(getTargetValue().x);
        upperArmTarget.set(getTargetValue().y);

        upperArm.periodic();
        lowerArm.periodic();
    }
}
