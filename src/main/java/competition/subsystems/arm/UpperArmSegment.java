package competition.subsystems.arm;

import javax.inject.Inject;
import javax.inject.Singleton;

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

    public final DoubleProperty absoluteEncoderProp;
    public final DoubleProperty neoPositionProp;

    @Inject
    public UpperArmSegment(XCANSparkMaxFactory sparkMaxFactory, XDutyCycleEncoder.XDutyCycleEncoderFactory dutyCycleEncoderFactory,
                           ElectricalContract eContract, PropertyFactory propFactory){
        super("UnifiedArmSubsystem/UpperArm", propFactory);
        String prefix = "UnifiedArmSubsystem/UpperArm";
        propFactory.setPrefix(prefix);
        this.contract = eContract;
        if(contract.isLowerArmReady()){
            this.leftMotor = sparkMaxFactory.create(eContract.getUpperArmLeftMotor(), prefix,"LeftMotor");
            this.rightMotor = sparkMaxFactory.create(eContract.getUpperArmRightMotor(), prefix,"RightMotor");

            leftMotor.follow(rightMotor, contract.getUpperArmLeftMotor().inverted);
        }
        if (contract.isUpperArmEncoderReady()) {
            this.absoluteEncoder = dutyCycleEncoderFactory.create(contract.getUpperArmEncoder());
        }

        setSoftLimit(false);

        absoluteEncoderProp = propFactory.createEphemeralProperty("AbsoluteEncoder", 0.0);
        neoPositionProp = propFactory.createEphemeralProperty("NeoPosition", 0.0);
    }


    @Override
    protected XCANSparkMax getLeaderMotor() {
        return rightMotor;
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
}