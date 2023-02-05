package competition.subsystems.arm;

import com.revrobotics.CANSparkMax;
import xbot.common.controls.actuators.XCANSparkMax;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

public abstract class ArmSegment {

    XCANSparkMax leaderMotor;

    private DoubleProperty upperLimitInDegrees;
    private DoubleProperty lowerLimitInDegrees;
    private DoubleProperty degreesPerMotorRotationProp;
    private double calibrationOffset;

    public ArmSegment(XCANSparkMax leaderMotor, String prefix, PropertyFactory propFactory) {
        this.leaderMotor = leaderMotor;
        propFactory.setPrefix(prefix);
        upperLimitInDegrees = propFactory.createPersistentProperty("upperLimitInDegrees", 0);
        lowerLimitInDegrees = propFactory.createPersistentProperty("lowerLimitInDegrees", 0);
    }

    public abstract boolean isMotorReady();
    public abstract boolean isAbsoluteEncoderReady();

    public void setPower(double power) {
        if (isMotorReady()) {
            leaderMotor.set(power);
        }
    }

    private

    private double getArmPositionInDegrees() {
        if (isAbsoluteEncoderReady()) {
            return leaderMotor.getPosition() * degreesPerMotorRotationProp.get() - calibrationOffset;
        }
        return 0;
    }

    private void setSoftLimit(boolean enabled) {
        if (isMotorReady()) {
            if (enabled) {
                enableSoftLimit(true);
                configSoftLimit(upperLimit.get(), lowerLimit.get());

            }
            enableSoftLimit(false);
        }
    }

    private void enableSoftLimit(boolean on){
        leaderMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward,on);
        leaderMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,on);
    }

    private void configSoftLimit(double upper, double lower){
        leaderMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward,(float)upper);
        leaderMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,(float)lower);
    }
}
