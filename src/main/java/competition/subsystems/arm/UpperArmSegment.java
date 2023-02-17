package competition.subsystems.arm;

import javax.inject.Inject;
import javax.inject.Singleton;

import com.revrobotics.CANSparkMax;
import competition.electrical_contract.ElectricalContract;
import xbot.common.controls.actuators.XCANSparkMax;
import xbot.common.controls.actuators.XCANSparkMax.XCANSparkMaxFactory;
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

    @Inject
    public UpperArmSegment(XCANSparkMaxFactory sparkMaxFactory, XDutyCycleEncoder.XDutyCycleEncoderFactory dutyCycleEncoderFactory,
                           ElectricalContract eContract, PropertyFactory propFactory){
        super("UnifiedArmSubsystem/UpperArm", propFactory, 90, -270);
        String prefix = "UnifiedArmSubsystem/UpperArm";
        propFactory.setPrefix(prefix);
        degreesPerMotorRotationProp = propFactory.createPersistentProperty("degreesPerMotorRotation",1.26);
        absoluteEncoderOffsetInDegreesProp = propFactory.createPersistentProperty("AbsoluteEncoderOffsetInDegrees", 0.0);

        this.contract = eContract;
        if(contract.isLowerArmReady()){
            this.leftMotor = sparkMaxFactory.create(eContract.getUpperArmLeftMotor(), prefix,"LeftMotor");
            this.rightMotor = sparkMaxFactory.create(eContract.getUpperArmRightMotor(), prefix,"RightMotor");

            leftMotor.follow(rightMotor, contract.getUpperArmLeftMotor().inverted);

            configureCommonMotorProperties();
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
    public void periodic() {
        super.periodic();
        if (isMotorReady()) {
            rightMotor.periodic();
        }
    }
}