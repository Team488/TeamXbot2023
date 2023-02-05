package competition.subsystems.arm;

import com.revrobotics.CANSparkMax;
import xbot.common.controls.actuators.XCANSparkMax;
import xbot.common.controls.sensors.XDutyCycleEncoder;
import xbot.common.properties.BooleanProperty;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

public abstract class ArmSegment {

    private final DoubleProperty upperLimitInDegrees;
    private final DoubleProperty lowerLimitInDegrees;
    private final DoubleProperty degreesPerMotorRotationProp;
    private final BooleanProperty useAbsoluteEncoderProp;
    private double motorEncoderOffsetInDegrees;
    private double absoluteEncoderOffsetInDegrees;

    public ArmSegment(String prefix, PropertyFactory propFactory) {
        propFactory.setPrefix(prefix);
        upperLimitInDegrees = propFactory.createPersistentProperty("upperLimitInDegrees", 0);
        lowerLimitInDegrees = propFactory.createPersistentProperty("lowerLimitInDegrees", 0);
        degreesPerMotorRotationProp = propFactory.createPersistentProperty("degreesPerMotorRotation", 360);
        useAbsoluteEncoderProp = propFactory.createPersistentProperty("useAbsoluteEncoder", false);
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

    private double getArmPositionFromAbsoluteEncoder() {
        if (isAbsoluteEncoderReady()) {
            return getAbsoluteEncoder().getAbsolutePosition().getDegrees() - absoluteEncoderOffsetInDegrees;
        }
        return 0;
    }

    private double getArmPositionFromMotorEncoder() {
        if (isMotorReady()) {
            return getLeaderMotor().getPosition() * degreesPerMotorRotationProp.get() - motorEncoderOffsetInDegrees;
        }
        return 0;
    }

    public void calibrateThisPositionAs(double degrees) {
        if (isAbsoluteEncoderReady()) {
            absoluteEncoderOffsetInDegrees = getAbsoluteEncoder().getAbsolutePosition().getDegrees() - degrees;
        }
        if (isMotorReady()) {
            motorEncoderOffsetInDegrees = getLeaderMotor().getPosition() * degreesPerMotorRotationProp.get() - degrees;
        }
    }

    public double getArmPositionInDegrees() {
        if (useAbsoluteEncoderProp.get()) {
            return getArmPositionFromAbsoluteEncoder();
        } else {
            return getArmPositionFromMotorEncoder();
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
                        (upperLimitInDegrees.get() + motorEncoderOffsetInDegrees) / degreesPerMotorRotationProp.get(),
                        (lowerLimitInDegrees.get() + motorEncoderOffsetInDegrees) / degreesPerMotorRotationProp.get());
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
}
