package competition.subsystems.arm;

import competition.electrical_contract.ElectricalContract;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import xbot.common.command.BaseSetpointSubsystem;
import xbot.common.controls.actuators.XSolenoid;
import xbot.common.math.ContiguousDouble;
import xbot.common.math.MathUtils;
import xbot.common.math.XYPair;
import xbot.common.properties.BooleanProperty;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public class UnifiedArmSubsystem extends BaseSetpointSubsystem<XYPair> {

    public LowerArmSegment lowerArm;
    public UpperArmSegment upperArm;
    public XSolenoid lowerArmBrakeSolenoid;
    private XYPair targetPosition;
    public final ArmPositionSolver solver;
    private final DoubleProperty lowerArmTarget;
    private final DoubleProperty upperArmTarget;
    private final DoubleProperty currentXPosition;
    private final DoubleProperty currentZPosition;
    private final DoubleProperty maximumXPosition;
    private final DoubleProperty maximumZPosition;
    private final DoubleProperty minimumZPosition;
    private GamePieceMode gamePieceMode;
    public enum GamePieceMode {
        Cone,
        Cube,
    }
    public enum KeyArmPosition {
        Ground,
        LoadingTray,
        LowGoal,
        MidGoal,
        HighGoal,
        FullyRetracted,
        AcquireFromCollector,
        SafeExternalTransition,
        StartingPosition
    }

    public enum RobotFacing {
        Forward,
        Backward
    }

    // Key positions for the end effector
    public static XYPair fullyRetractedPosition = new XYPair(0, 0);
    public static XYPair lowerGoalPosition = new XYPair(1*12, 0);
    public static XYPair midGoalPosition = new XYPair(3*12, 2*12);
    public static XYPair highGoalPosition = new XYPair(4*12, 3*12);

    // Key angles for the lower and upper arms (in degrees)
    public static XYPair fullyRetractedAngles = new XYPair(90, 0);
    public static XYPair lowerGoalCubeAngles = new XYPair(66, 33);
    public static XYPair midGoalCubeAngles = new XYPair(77, 68);
    public static XYPair highGoalCubeAngles = new XYPair(55, 121);
    public static XYPair lowerGoalConeAngles = new XYPair(49, 35);
    public static XYPair midGoalConeAngles = new XYPair(82, 80);
    public static XYPair highGoalConeAngles = new XYPair(60, 130);
    public static XYPair acquireFromCollectorAngles = new XYPair(
            75,
            ArmPositionSolver.convertOldArmAngleToNewArmAngle(75,-90));
    public static XYPair safeExternalTransitionAngles = new XYPair(
            100,
            90);
    // :todo add correct values for all the angles
    public static XYPair groundAngle = new XYPair(45, 28);
    public static XYPair loadingTrayAngle = new XYPair(103, 51);
    public static XYPair startingPositionAngles = new XYPair(110, 20);

    double testRangeRadians = 0.17453292519943295; // 10 degrees

    protected final BooleanProperty calibratedProp;

    protected final BooleanProperty areBrakesEngaged;

    protected final BooleanProperty brakeDisabled;

    @Inject
    public UnifiedArmSubsystem(
            LowerArmSegment lowerArm,
            UpperArmSegment upperArm,
            XSolenoid.XSolenoidFactory xSolenoidFactory,
            ElectricalContract eContract,
            PropertyFactory pf) {
        this.lowerArm = lowerArm;
        this.upperArm = upperArm;
        this.lowerArmBrakeSolenoid = xSolenoidFactory.create(eContract.getLowerArmBrakeSolenoid().channel);
        ArmPositionSolverConfiguration armConfig = new ArmPositionSolverConfiguration(
            44.5,
            33.0,
            Rotation2d.fromDegrees(15.0),
            Rotation2d.fromDegrees(165.0),
            Rotation2d.fromDegrees(-175),
            Rotation2d.fromDegrees(-5.0)
        );
        solver = new ArmPositionSolver(armConfig);
        pf.setPrefix(this);
        calibratedProp = pf.createEphemeralProperty("Calibrated", true);
        upperArmTarget = pf.createEphemeralProperty("UpperArmTarget", 0.0);
        lowerArmTarget = pf.createEphemeralProperty("LowerArmTarget", 0.0);
        areBrakesEngaged = pf.createEphemeralProperty("AreBrakesEngaged", false);
        brakeDisabled = pf.createEphemeralProperty("Brake disable override", false);
        targetPosition = getCurrentValue();
        currentXPosition = pf.createEphemeralProperty("Current X Position", 0.0);
        currentZPosition = pf.createEphemeralProperty("Current Z Position", 0.0);

        // Maximum extents based on frame perimeter being 15in from lower arm joint, joint being 8in above ground.
        // Rules are: Max height: 6ft6in (78in), max extension 48in. Using 2 inches as buffer space since position
        // measurements are to the clamp on the end effector.
        maximumXPosition = pf.createPersistentProperty("Maximum X Position", 15 + 48 - 2);
        maximumZPosition = pf.createPersistentProperty("Maximum Z Position", 78 - 8 - 2);
        minimumZPosition = pf.createPersistentProperty("Minimum Z Position", -2);

        areBrakesEngaged.set(true);
    }

    public XYPair getKeyArmCoordinates(KeyArmPosition keyArmPosition, RobotFacing facing){
        XYPair candidate = new XYPair();
        switch (keyArmPosition){
            case LowGoal:
                candidate = lowerGoalPosition;
                break;

            case MidGoal:
                candidate = midGoalPosition;
                break;

            case HighGoal:
                candidate=highGoalPosition;
                break;

            case FullyRetracted:
                candidate = fullyRetractedPosition;
                break;

            default:
                break;
        }
        if (facing == RobotFacing.Backward){
            // TODO: mirror the coordinates
        }
        return candidate;
    }

    public XYPair getKeyArmAngles(KeyArmPosition keyArmPosition, RobotFacing facing) {
        XYPair candidate = new XYPair();
        switch (keyArmPosition) {
            case LowGoal:
                if(gamePieceMode == GamePieceMode.Cube){
                    candidate = lowerGoalCubeAngles;
                }
                else{
                    candidate = lowerGoalConeAngles;
                }
                break;

            case MidGoal:
                if(gamePieceMode == GamePieceMode.Cube){
                    candidate = midGoalCubeAngles;
                }
                else{
                    candidate = midGoalConeAngles;
                }
                break;
            case HighGoal:
                if(gamePieceMode == GamePieceMode.Cube){
                    candidate = highGoalCubeAngles;
                }
                else{
                    candidate = highGoalConeAngles;
                }
                break;

            case Ground:
                candidate = groundAngle;
                break;
            case LoadingTray:
                candidate = loadingTrayAngle;
                break;

            case FullyRetracted:
                candidate = fullyRetractedAngles;
                break;
            case AcquireFromCollector:
                candidate = acquireFromCollectorAngles;
                break;
            case SafeExternalTransition:
                candidate = safeExternalTransitionAngles;
                break;
            case StartingPosition:
                candidate = startingPositionAngles;
                break;
            default:
                break;
        }
        if (facing == RobotFacing.Backward) {
            candidate = mirrorArmAngles(candidate);
        }
        return candidate;
    }

    public RobotFacing getCurrentEndEffectorFacing() {
        if (getCurrentXZCoordinates().x > 0) {
            return RobotFacing.Forward;
        }
        else {
            return RobotFacing.Backward;
        }
    }

    public static XYPair mirrorArmAngles(XYPair angles) {
        // The lower arm needs to be mirrored around 90 degrees.
        // The upper arm needs to be mirrored around 0 degrees.
        return new XYPair(180 - angles.x, -angles.y);
    }

    @Override
    public XYPair getCurrentValue() {

        return new XYPair(
                lowerArm.getArmPositionInDegrees(),
                upperArm.getArmPositionInDegrees()
        );
        // Eventually do the smart thing. For now, just do angles.
        /*
        return solver.getPositionFromRadians(
            lowerArm.getArmPositionInRadians(),
            upperArm.getArmPositionInRadians());

         */
    }

    public XYPair getCurrentXZCoordinates() {
        return solver.getPositionFromRadians(
            lowerArm.getArmPositionInRadians(),
            upperArm.getArmPositionInRadians());
    }

    public XYPair constrainXZPosition(XYPair targetPosition) {
        double maximumX = maximumXPosition.get();
        double maximumZ = maximumZPosition.get();
        double minimumZ = minimumZPosition.get();
        return new XYPair(
                MathUtils.constrainDouble(targetPosition.x, -maximumX, maximumX),
                MathUtils.constrainDouble(targetPosition.y, minimumZ, maximumZ));
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

        boolean encodersPluggedIn = areEncodersPluggedIn();

        if (!encodersPluggedIn) {
            log.error("Disabling arm control.");
            setIsCalibrated(false);
            lowerArm.setPower(0);
            upperArm.setPower(0);
        } else {
            // Encoders are working, so we can move the arms.
            if (areBrakesEngaged.get() && !getDisableBrake()) {
                lowerArm.setPower(0);
            } else {
                lowerArm.setArmToAngle(lowerArmAngle);
            }
            upperArm.setArmToAngle(upperArmAngle);
        }
    }

    private boolean areEncodersPluggedIn() {
        // If our absolute encoders have come unplugged, then we are about to have a very bad day.
        // When they become unplugged, they return a native value of 0. This means that any read of the
        // robot angle will instead return the negated offset.
        // So, we can check for very close equality between detected angle and negated offset. This should never
        // trigger in normal operation, as live robot values have very long tails (e.g. 8.38739834598374858 degrees).
        var reboundedLowerArmAngle = ContiguousDouble.reboundValue(lowerArm.getArmPositionInDegrees(), 0, 360);
        var reboundedNegatedLowerArmOffset = ContiguousDouble.reboundValue(-lowerArm.getAbsoluteEncoderOffsetInDegrees(), 0, 360);
        var reboundedUpperArmAngle = ContiguousDouble.reboundValue(upperArm.getArmPositionInDegrees(), 0, 360);
        var reboundedNegatedUpperArmOffset = ContiguousDouble.reboundValue(-upperArm.getAbsoluteEncoderOffsetInDegrees(), 0, 360);

        boolean encodersPluggedIn = true;
        if (Math.abs(reboundedLowerArmAngle - reboundedNegatedLowerArmOffset) < 0.000001) {
            log.error("The lower arm absolute encoder has come unplugged.");
            encodersPluggedIn = false;
        }
        if (Math.abs(reboundedUpperArmAngle - reboundedNegatedUpperArmOffset) < 0.000001) {
            log.error("The upper arm absolute encoder has come unplugged.");
            encodersPluggedIn = false;
        }
        return encodersPluggedIn;
    }

    @Override
    public boolean isCalibrated() {
        return calibratedProp.get();
    }

    public void setPitchCompensation(boolean enabled) {
        this.lowerArm.setPitchCompensation(enabled);
        // No need to pitch compensate the upper arm now that it's linked to the lower arm
        // (no longer a four-bar)
    }

    public void setIsCalibrated(boolean isCalibrated) {
        calibratedProp.set(isCalibrated);
    }

    public Command createForceUncalibratedCommand() {
        return new InstantCommand(() -> {
            log.info("Forcing arms to uncalibrated mode. Only manual operation will be respected.");
            calibratedProp.set(false);
        });
    }

    /**
     * Adjusts the target lower arm angle by some number of degrees.
     * @param trimAmount The number of degrees to change the target by.
     * @return A command that changes the target.
     */
    public Command createLowerArmTrimCommand(double trimAmount) {
        return new InstantCommand(() -> {
            XYPair currentValue = getCurrentValue();
            setTargetValue(new XYPair(currentValue.x + trimAmount, currentValue.y));
        });
    }

    /**
     * Adjusts the target upper arm angle by some number of degrees.
     * @param trimAmount The number of degrees to change the target by.
     * @return A command that changes the target.
     */
    public Command createUpperArmTrimCommand(double trimAmount) {
        return new InstantCommand(() -> {
            XYPair currentValue = getCurrentValue();
            setTargetValue(new XYPair(currentValue.x, currentValue.y + trimAmount));
        });
    }

    public boolean areBrakesEngaged() {
        return areBrakesEngaged.get();
    }

    public void setDisableBrake(boolean disabled) {
        brakeDisabled.set(disabled);
    }

    public boolean getDisableBrake() {
        return brakeDisabled.get();
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
    public void setBrake(boolean on){
        if(on){
            lowerArmBrakeSolenoid.setOn(false);
            areBrakesEngaged.set(true);
        } else {
            lowerArmBrakeSolenoid.setOn(true);
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

        XYPair currentPosition = getCurrentXZCoordinates();
        currentXPosition.set(currentPosition.x);
        currentZPosition.set(currentPosition.y);
    }

    public void setGamePieceMode(GamePieceMode gamePiece){
        this.gamePieceMode = gamePiece;
    }

    public Command createSetGamePieceModeCommand(GamePieceMode gamePiece){
        return new InstantCommand(() -> setGamePieceMode(gamePiece));
    }
}
