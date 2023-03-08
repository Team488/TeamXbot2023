package competition.subsystems.arm;

import javax.inject.Inject;
import javax.inject.Singleton;

import com.revrobotics.CANSparkMax;
import competition.electrical_contract.ElectricalContract;
import competition.subsystems.pose.PoseSubsystem;
import xbot.common.controls.actuators.XCANSparkMax;
import xbot.common.controls.actuators.XCANSparkMax.XCANSparkMaxFactory;
import xbot.common.controls.actuators.XCANSparkMaxPIDProperties;
import xbot.common.controls.sensors.XDutyCycleEncoder;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

@Singleton
public class UpperArmSegment extends ArmSegment {
    public XCANSparkMax leftMotor;
    public XCANSparkMax rightMotor;
    public XDutyCycleEncoder absoluteEncoder;
    public ElectricalContract contract;
    public DoubleProperty powerProp;

    public  DoubleProperty extendLimit;
    public  DoubleProperty retractLimit;
    private final DoubleProperty degreesPerMotorRotationProp;
    private final DoubleProperty absoluteEncoderOffsetInDegreesProp;
    private final DoubleProperty lowerLimitInDegrees;
    private final DoubleProperty upperLimitInDegrees;

    @Inject
    public UpperArmSegment(XCANSparkMaxFactory sparkMaxFactory, XDutyCycleEncoder.XDutyCycleEncoderFactory dutyCycleEncoderFactory,
                           ElectricalContract eContract, PropertyFactory propFactory, PoseSubsystem pose){
        super("UnifiedArmSubsystem/UpperArm", propFactory, pose, 180, -180);
        String prefix = "UnifiedArmSubsystem/UpperArm";
        propFactory.setPrefix(prefix);
        degreesPerMotorRotationProp = propFactory.createPersistentProperty("degreesPerMotorRotation",1.26);
        absoluteEncoderOffsetInDegreesProp = propFactory.createPersistentProperty("AbsoluteEncoderOffsetInDegrees", -153.6);
        lowerLimitInDegrees = propFactory.createPersistentProperty("LowerLimitInDegrees", -1000);
        upperLimitInDegrees = propFactory.createPersistentProperty("UpperLimitInDegrees", 1000);

        XCANSparkMaxPIDProperties motorPidDefaults = new XCANSparkMaxPIDProperties(
                0.06, // P
                0.0, // I
                0, // D
                0, // IZone
                0, // FF
                0.75, // MaxOutput
                -0.75 // MinOutput
        );

        this.contract = eContract;
        if(contract.isLowerArmReady()){
            this.leftMotor = sparkMaxFactory.create(eContract.getUpperArmLeftMotor(), prefix,"LeftMotor");
            this.rightMotor = sparkMaxFactory.create(eContract.getUpperArmRightMotor(), prefix,"RightMotor", motorPidDefaults);

            leftMotor.follow(rightMotor, contract.getUpperArmLeftMotor().inverted);

            configureCommonMotorProperties();

            leftMotor.setSmartCurrentLimit(30);
            rightMotor.setSmartCurrentLimit(30);
        }
        if (contract.isUpperArmEncoderReady()) {
            this.absoluteEncoder = dutyCycleEncoderFactory.create(contract.getUpperArmEncoder());
        }

        setSoftLimit(false);
    }
    @Override
    protected double getAbsoluteEncoderOffsetInDegrees() {
        return absoluteEncoderOffsetInDegreesProp.get();
    }

    @Override
    public void setAbsoluteEncoderOffsetInDegrees(double offset) {
        absoluteEncoderOffsetInDegreesProp.set(offset);
    }

    @Override
    protected double getDegreesPerMotorRotation() {
        return degreesPerMotorRotationProp.get();
    }

    @Override
    protected XCANSparkMax getLeaderMotor() {
        return rightMotor;
    }

    @Override
    protected XCANSparkMax getFollowerMotor() {
        return leftMotor;
    }

    @Override
    protected XDutyCycleEncoder getAbsoluteEncoder() {
        return absoluteEncoder;
    }

    @Override
    public boolean isMotorReady() {
        return contract.isLowerArmReady();
    }

    @Override
    public boolean isAbsoluteEncoderReady() {
        return contract.isLowerArmEncoderReady();
    }

    @Override
    protected double getUpperLimitInDegrees() {
        return upperLimitInDegrees.get();
    }

    @Override
    protected double getLowerLimitInDegrees() {
        return lowerLimitInDegrees.get();
    }

    @Override
    protected double getVoltageOffset() {
        return 0;
    }

    @Override
    public void periodic() {
        super.periodic();
        if (isMotorReady()) {
            rightMotor.periodic();
        }
    }
}