package competition.subsystems.arm;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import xbot.common.controls.actuators.XCANSparkMax;
import xbot.common.controls.sensors.XDutyCycleEncoder;
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
    private double absoluteEncoderOffsetInDegrees;

    private final DoubleProperty absoluteEncoderPositionProp;
    private final DoubleProperty neoPositionProp;

    public ArmSegment(String prefix, PropertyFactory propFactory) {
        propFactory.setPrefix(prefix);
        upperLimitInDegrees = propFactory.createPersistentProperty("upperLimitInDegrees", 0);
        lowerLimitInDegrees = propFactory.createPersistentProperty("lowerLimitInDegrees", 0);
        useAbsoluteEncoderProp = propFactory.createPersistentProperty("useAbsoluteEncoder", false);
        absoluteEncoderPositionProp = propFactory.createEphemeralProperty("AbsoluteEncoderPosition", 0.0);
        neoPositionProp = propFactory.createEphemeralProperty("NeoPosition", 0.0);
    }

    protected abstract XCANSparkMax getLeaderMotor();
    protected abstract XDutyCycleEncoder getAbsoluteEncoder();

    public abstract boolean isMotorReady();
    public abstract boolean isAbsoluteEncoderReady();

    public void setPower(double power) {
        if (isMotorReady()) {
            getLeaderMotor().set(power);
        }
    }

    private double getArmPositionFromAbsoluteEncoderInDegrees() {
        if (isAbsoluteEncoderReady()) {
            return getAbsoluteEncoder().getAbsolutePosition().getDegrees() - absoluteEncoderOffsetInDegrees;
        }
        return 0;
    }

    private double getArmPositionFromMotorEncoderInDegrees() {
        if (isMotorReady()) {
            return getLeaderMotor().getPosition() * getDegreesPerMotorRotation() - motorEncoderOffsetInDegrees;
        }
        return 0;
    }

    public void calibrateThisPositionAs(double degrees) {
        if (isAbsoluteEncoderReady()) {
            absoluteEncoderOffsetInDegrees = getAbsoluteEncoder().getAbsolutePosition().getDegrees() - degrees;
        }
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
                configSoftLimit(
                        (upperLimitInDegrees.get() + motorEncoderOffsetInDegrees) / getDegreesPerMotorRotation(),
                        (lowerLimitInDegrees.get() + motorEncoderOffsetInDegrees) / getDegreesPerMotorRotation());
            }
            enableSoftLimit(false);
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
        // We want to use the absolute encoder to figure out how far away we are from the target angle. However, the motor
        // controller will be using its internal encoder, so we need to translate from one to the other.
        // Our approach is:
        // 1) Determine the difference between the goal and our absolute position
        // 2) Translate that difference, the "delta", into motor rotations
        // 3) Read the current motor position in rotations
        // 4) Add the delta to the current position to get a goal position in units the motor controller understands

        if (isAbsoluteEncoderReady() && isMotorReady()) {
            double delta = WrappedRotation2d.fromDegrees(angle.getDegrees() - getArmPositionFromAbsoluteEncoderInDegrees()).getDegrees();
            double deltaInMotorRotations = delta / getDegreesPerMotorRotation();
            double goalPosition = deltaInMotorRotations + getLeaderMotor().getPosition();;
            getLeaderMotor().setReference(goalPosition, CANSparkMax.ControlType.kPosition);
        }
    }

    public void periodic() {
        absoluteEncoderPositionProp.set(getAbsoluteEncoder().getWrappedPosition().getDegrees());
        neoPositionProp.set(getLeaderMotor().getPosition());
    }
}
