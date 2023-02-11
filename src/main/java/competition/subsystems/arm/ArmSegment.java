package competition.subsystems.arm;

import com.revrobotics.CANSparkMax;
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

    private final DoubleProperty upperLimitInDegrees;
    private final DoubleProperty lowerLimitInDegrees;
    protected abstract double getDegreesPerMotorRotation();
    private final BooleanProperty useAbsoluteEncoderProp;
    private double motorEncoderOffsetInDegrees;
    protected abstract double getAbsoluteEncoderOffsetInDegrees();
    private final DoubleProperty absoluteEncoderPositionProp;
    private final DoubleProperty neoPositionProp;
    private final DoubleProperty neoPositionInDegreesProp;

    private static Logger log = LogManager.getLogger(ArmSegment.class);

    String prefix = "";

    double upperDegreeReference;
    double lowerDegreeReference;

    public ArmSegment(String prefix, PropertyFactory propFactory, double upperDegreeReference, double lowerDegreeReference) {
        propFactory.setPrefix(prefix);
        this.prefix= prefix;
        this.upperDegreeReference = upperDegreeReference;
        this.lowerDegreeReference = lowerDegreeReference;
        upperLimitInDegrees = propFactory.createPersistentProperty("upperLimitInDegrees", 0);
        lowerLimitInDegrees = propFactory.createPersistentProperty("lowerLimitInDegrees", 0);
        useAbsoluteEncoderProp = propFactory.createPersistentProperty("useAbsoluteEncoder", true);
        absoluteEncoderPositionProp = propFactory.createEphemeralProperty("AbsoluteEncoderPosition", 0.0);
        neoPositionProp = propFactory.createEphemeralProperty("NeoPosition", 0.0);
        neoPositionInDegreesProp = propFactory.createEphemeralProperty("NeoPositionInDegrees", 0.0);
    }

    protected abstract XCANSparkMax getLeaderMotor();
    protected abstract XDutyCycleEncoder getAbsoluteEncoder();

    public abstract boolean isMotorReady();
    public abstract boolean isAbsoluteEncoderReady();

    public void setPower(double power) {
        if (isMotorReady()) {

            // if too high, no more positive power
            double currentAngle = getArmPositionFromAbsoluteEncoderInDegrees();
            if (currentAngle > upperLimitInDegrees.get())
            {
                power = MathUtils.constrainDouble(power, -1, 0);
            }
            // if too low, no more negative power.
            if (currentAngle < lowerLimitInDegrees.get()) {
                power = MathUtils.constrainDouble(power, 0, 1);
            }

            getLeaderMotor().set(power);
        }
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
        if (isMotorReady()) {
            motorEncoderOffsetInDegrees = getLeaderMotor().getPosition() * getDegreesPerMotorRotation() - degrees;
        }
    }

    public double getArmPositionInDegrees() {
        if (useAbsoluteEncoderProp.get()) {
            return getArmPositionFromAbsoluteEncoderInDegrees();
        } else {
            return getArmPositionFromMotorEncoderInDegrees();
        }
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

                double upperLimitInRotations = (upperLimitInDegrees.get() + motorEncoderOffsetInDegrees) / getDegreesPerMotorRotation();
                double lowerLimitInRotations = (lowerLimitInDegrees.get() + motorEncoderOffsetInDegrees) / getDegreesPerMotorRotation();
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

    public void setArmToAngle(Rotation2d angle) {

        // Coerce angle to a safe angle
        double targetAngleDegrees =
                MathUtils.constrainDouble(angle.getDegrees(), lowerLimitInDegrees.get(), upperLimitInDegrees.get());

        // We want to use the absolute encoder to figure out how far away we are from the target angle. However, the motor
        // controller will be using its internal encoder, so we need to translate from one to the other.
        // Our approach is:
        // 1) Determine the difference between the goal and our absolute position
        // 2) Translate that difference, the "delta", into motor rotations
        // 3) Read the current motor position in rotations
        // 4) Add the delta to the current position to get a goal position in units the motor controller understands

        if (isAbsoluteEncoderReady() && isMotorReady()) {
            double delta = WrappedRotation2d.fromDegrees(targetAngleDegrees - getArmPositionFromAbsoluteEncoderInDegrees()).getDegrees();
            double deltaInMotorRotations = delta / getDegreesPerMotorRotation();
            double goalPosition = deltaInMotorRotations + getLeaderMotor().getPosition();
            getLeaderMotor().setReference(goalPosition, CANSparkMax.ControlType.kPosition);
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
    }
}
