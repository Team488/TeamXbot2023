package competition.subsystems.arm;

import com.revrobotics.CANSparkMax;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import org.apache.log4j.LogManager;
import org.apache.log4j.Logger;
import xbot.common.controls.actuators.XCANSparkMax;
import xbot.common.controls.sensors.XDutyCycleEncoder;
import xbot.common.math.ContiguousDouble;
import xbot.common.math.MathUtils;
import xbot.common.math.WrappedRotation2d;
import xbot.common.properties.BooleanProperty;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

public abstract class ArmSegment {

    protected abstract double getDegreesPerMotorRotation();
    private final BooleanProperty useAbsoluteEncoderProp;
    private final BooleanProperty usePitchCompensationProp;
    private final BooleanProperty invertPitchCompensationProp;
    private double motorEncoderOffsetInDegrees;
    protected abstract double getAbsoluteEncoderOffsetInDegrees();
    public abstract void setAbsoluteEncoderOffsetInDegrees(double offset);
    private final DoubleProperty absoluteEncoderPositionProp;
    private final DoubleProperty neoPositionProp;
    private final DoubleProperty neoPositionInDegreesProp;
    private final DoubleProperty compensatedPositionProp;
    private final PoseSubsystem pose;

    private static Logger log = LogManager.getLogger(ArmSegment.class);

    String prefix = "";

    double upperDegreeReference;
    double lowerDegreeReference;

    public ArmSegment(String prefix, PropertyFactory propFactory, PoseSubsystem pose, double upperDegreeReference, double lowerDegreeReference) {
        propFactory.setPrefix(prefix);
        this.prefix= prefix;
        this.pose = pose;
        this.upperDegreeReference = upperDegreeReference;
        this.lowerDegreeReference = lowerDegreeReference;
        useAbsoluteEncoderProp = propFactory.createPersistentProperty("useAbsoluteEncoder", true);
        usePitchCompensationProp = propFactory.createEphemeralProperty("usePitchCompensation", false);
        usePitchCompensationProp.set(false);
        invertPitchCompensationProp = propFactory.createPersistentProperty("invertPitchCompensation", false);
        absoluteEncoderPositionProp = propFactory.createEphemeralProperty("AbsoluteEncoderPosition", 0.0);
        neoPositionProp = propFactory.createEphemeralProperty("NeoPosition", 0.0);
        neoPositionInDegreesProp = propFactory.createEphemeralProperty("NeoPositionInDegrees", 0.0);
        compensatedPositionProp = propFactory.createEphemeralProperty("CompensatedPosition", 0.0);
    }

    protected void configureCommonMotorProperties() {
        getLeaderMotor().setOpenLoopRampRate(0.05);
        getLeaderMotor().setClosedLoopRampRate(0.05);
        getLeaderMotor().setIdleMode(CANSparkMax.IdleMode.kBrake);
        getFollowerMotor().setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    protected abstract XCANSparkMax getLeaderMotor();
    protected abstract XCANSparkMax getFollowerMotor();
    protected abstract XDutyCycleEncoder getAbsoluteEncoder();

    public abstract boolean isMotorReady();
    public abstract boolean isAbsoluteEncoderReady();

    protected abstract double getUpperLimitInDegrees();
    protected abstract double getLowerLimitInDegrees();
    protected abstract double getVoltageOffset();
    protected abstract void setUpperLimitInDegrees(double upperLimitInDegrees);
    protected abstract void setLowerLimitInDegrees(double lowerLimitInDegrees);

    public void setPower(double power) {
        if (isMotorReady()) {

            // if too high, no more positive power
            double currentAngle = getArmPositionInDegrees();
            if (currentAngle > getUpperLimitInDegrees())
            {
                power = MathUtils.constrainDouble(power, -1, 0);
            }
            // if too low, no more negative power.
            if (currentAngle < getLowerLimitInDegrees()) {
                power = MathUtils.constrainDouble(power, 0, 1);
            }

            getLeaderMotor().set(power);
        }
    }

    public boolean isAboveUpperLimit() {
        return getArmPositionInDegrees() > getUpperLimitInDegrees();
    }

    public boolean isBelowLowerLimit() {
        return getArmPositionInDegrees() < getLowerLimitInDegrees();
    }

    public void setPitchCompensation(boolean enabled) {
        this.usePitchCompensationProp.set(enabled);
    }

    public double getArmPositionFromAbsoluteEncoderInDegrees() {
        if (isAbsoluteEncoderReady()) {
            return ContiguousDouble.reboundValue(
                    getAbsoluteEncoder().getAbsoluteDegrees()- getAbsoluteEncoderOffsetInDegrees(),
                    lowerDegreeReference,
                    upperDegreeReference);
        }
        return 0;
    }

    public double getArmPositionFromMotorEncoderInDegrees() {
        if (isMotorReady()) {
            return getLeaderMotor().getPosition() * getDegreesPerMotorRotation() - motorEncoderOffsetInDegrees;
        }
        return 0;
    }

    public void calibrateThisPositionAs(double degrees) {
        setAbsoluteEncoderOffsetInDegrees(getAbsoluteEncoder().getAbsoluteDegrees() - degrees);
    }

    public double getArmPositionInDegrees() {
        double sensorPosition;
        if (useAbsoluteEncoderProp.get()) {
            sensorPosition = getArmPositionFromAbsoluteEncoderInDegrees();
        } else {
            sensorPosition = getArmPositionFromMotorEncoderInDegrees();
        }

        if (usePitchCompensationProp.get() == true) {
            return sensorPosition - (pose.getRobotPitch() * (invertPitchCompensationProp.get() ? -1.0 : 1.0));
        }

        return sensorPosition;
    }

    public double getArmPositionInRadians() {
        return Math.toRadians(getArmPositionInDegrees());
    }

    public void setSoftLimit(boolean enabled) {
        if (isMotorReady()) {
            if (enabled) {
                enableSoftLimit(true);
                // When setting the soft limits, remember that the underlying motor wants units of rotation, and that
                // we are potentially offset. So we need to figure out our actual degree target, then divide by
                // degrees per motor rotation to get something the SparkMAX can understand.

                double upperLimitInRotations = (getUpperLimitInDegrees() + motorEncoderOffsetInDegrees) / getDegreesPerMotorRotation();
                double lowerLimitInRotations = (getLowerLimitInDegrees() + motorEncoderOffsetInDegrees) / getDegreesPerMotorRotation();
                log.info(prefix + ", UpperLimitRotations:"+upperLimitInRotations+", LowerLimitRotation:"+lowerLimitInRotations);
                configSoftLimit(upperLimitInRotations, lowerLimitInRotations);
            } else {
                enableSoftLimit(false);
            }
        }
    }

    private void enableSoftLimit(boolean on){
        getLeaderMotor().enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward,on);
        getLeaderMotor().enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,on);
    }

    private void configSoftLimit(double upper, double lower){
        getLeaderMotor().setSoftLimit(CANSparkMax.SoftLimitDirection.kForward,(float)upper);
        getLeaderMotor().setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,(float)lower);
    }

    public double coerceAngleWithinLimits(double angle) {
        return MathUtils.constrainDouble(angle, getLowerLimitInDegrees(), getUpperLimitInDegrees());
    }

    public boolean isAngleWithinLimits(double angle) {
        return angle >= getLowerLimitInDegrees() && angle <= getUpperLimitInDegrees();
    }

    public void setArmToAngle(Rotation2d angle) {

        // Coerce angle to a safe angle.
        // Should already be done by the UnifiedArm, but just in case.
        double targetAngleDegrees = coerceAngleWithinLimits(angle.getDegrees());

        // We want to use the absolute encoder to figure out how far away we are from the target angle. However, the motor
        // controller will be using its internal encoder, so we need to translate from one to the other.
        // Our approach is:
        // 1) Determine the difference between the goal and our absolute position
        // 2) Translate that difference, the "delta", into motor rotations
        // 3) Read the current motor position in rotations
        // 4) Add the delta to the current position to get a goal position in units the motor controller understands

        if (isAbsoluteEncoderReady() && isMotorReady()) {
            double delta = WrappedRotation2d.fromDegrees(targetAngleDegrees - getArmPositionInDegrees()).getDegrees();
            double deltaInMotorRotations = delta / getDegreesPerMotorRotation();
            double goalPosition = deltaInMotorRotations + getLeaderMotor().getPosition();
            getLeaderMotor().setReference(
                    goalPosition,
                    CANSparkMax.ControlType.kPosition,
                    0,
                    getVoltageOffset());
        }
    }

    public void periodic() {
        if (isAbsoluteEncoderReady()) {
            absoluteEncoderPositionProp.set(getArmPositionFromAbsoluteEncoderInDegrees());
        }

        if (isMotorReady()) {
            neoPositionProp.set(getLeaderMotor().getPosition());
            neoPositionInDegreesProp.set(getArmPositionFromMotorEncoderInDegrees());
        }

        compensatedPositionProp.set(getArmPositionInDegrees());
    }
}
