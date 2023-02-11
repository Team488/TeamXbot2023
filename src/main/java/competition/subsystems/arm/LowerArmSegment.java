package competition.subsystems.arm;

import javax.inject.Inject;
import javax.inject.Singleton;

import com.revrobotics.CANSparkMax;
import competition.electrical_contract.ElectricalContract;
import edu.wpi.first.networktables.DoubleEntry;
import xbot.common.controls.actuators.XCANSparkMax;
import xbot.common.controls.actuators.XCANSparkMax.XCANSparkMaxFactory;
import xbot.common.controls.sensors.XDutyCycleEncoder;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

@Singleton
public class LowerArmSegment extends ArmSegment {
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
    public LowerArmSegment(XCANSparkMaxFactory sparkMaxFactory, XDutyCycleEncoder.XDutyCycleEncoderFactory dutyCycleEncoderFactory,
                           ElectricalContract eContract, PropertyFactory propFactory){
        super("UnifiedArmSubsystem/LowerArm", propFactory, 270, -90);
        String prefix = "UnifiedArmSubsystem/LowerArm";

        propFactory.setPrefix(prefix);
        degreesPerMotorRotationProp = propFactory.createPersistentProperty("degreesPerMotorRotation", 4.22);

        absoluteEncoderOffsetInDegreesProp = propFactory.createPersistentProperty("AbsoluteEncoderOffsetInDegrees", 0.0);

        this.contract = eContract;
        if(contract.isLowerArmReady()){
            this.leftMotor = sparkMaxFactory.create(eContract.getLowerArmLeftMotor(), prefix,"LeftMotor");
            this.rightMotor = sparkMaxFactory.create(eContract.getLowerArmRightMotor(), prefix,"RightMotor");

            leftMotor.follow(rightMotor, contract.getLowerArmLeftMotor().inverted);

            rightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
            leftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

            rightMotor.setOpenLoopRampRate(0.05);
            rightMotor.setClosedLoopRampRate(0.05);
        }
        if (contract.isLowerArmEncoderReady()) {
            this.absoluteEncoder = dutyCycleEncoderFactory.create(contract.getLowerArmEncoder());
        }

        setSoftLimit(false);
    }


    @Override
    protected double getDegreesPerMotorRotation() {
        return degreesPerMotorRotationProp.get();
    }

    @Override
    protected double getAbsoluteEncoderOffsetInDegrees() {
        return absoluteEncoderOffsetInDegreesProp.get();
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

    @Override
    public void periodic() {
        super.periodic();
        rightMotor.periodic();
    }
}