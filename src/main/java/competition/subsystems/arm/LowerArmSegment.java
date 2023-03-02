package competition.subsystems.arm;

import javax.inject.Inject;
import javax.inject.Singleton;

import com.revrobotics.CANSparkMax;
import competition.electrical_contract.ElectricalContract;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.networktables.DoubleEntry;
import xbot.common.controls.actuators.XCANSparkMax;
import xbot.common.controls.actuators.XCANSparkMax.XCANSparkMaxFactory;
import xbot.common.controls.actuators.XCANSparkMaxPIDProperties;
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

    private final DoubleProperty lowerLimitInDegrees;
    private final DoubleProperty upperLimitInDegrees;

    private final DoubleProperty voltageOffsetProp;


    @Inject
    public LowerArmSegment(XCANSparkMaxFactory sparkMaxFactory, XDutyCycleEncoder.XDutyCycleEncoderFactory dutyCycleEncoderFactory,
                           ElectricalContract eContract, PropertyFactory propFactory, PoseSubsystem pose){
        super("UnifiedArmSubsystem/LowerArm", propFactory, pose, 270, -90);
        String prefix = "UnifiedArmSubsystem/LowerArm";

        // TODO: Right now the max/min output is asymmetric (only works for the front side of the machine).
        // This will likely cause bad behavior on the back of the robot.
        XCANSparkMaxPIDProperties motorPidDefaults = new XCANSparkMaxPIDProperties(
                0.11, // P
                0.0001, // I
                0, // D
                0.0001, // IZone - basically disabling with a value this low
                0, // FF
                0.25, // MaxOutput
                -0.1 // MinOutput
        );

        propFactory.setPrefix(prefix);
        degreesPerMotorRotationProp = propFactory.createPersistentProperty("degreesPerMotorRotation", 4.22);
        absoluteEncoderOffsetInDegreesProp = propFactory.createPersistentProperty("AbsoluteEncoderOffsetInDegrees", -160.64561);
        lowerLimitInDegrees = propFactory.createPersistentProperty("LowerLimitInDegrees", 35);
        upperLimitInDegrees = propFactory.createPersistentProperty("UpperLimitInDegrees", 100);
        voltageOffsetProp = propFactory.createPersistentProperty("VoltageOffset", 1.0);

        this.contract = eContract;
        if(contract.isLowerArmReady()){
            this.leftMotor = sparkMaxFactory.create(eContract.getLowerArmLeftMotor(), prefix,"LeftMotor");
            this.rightMotor = sparkMaxFactory.create(eContract.getLowerArmRightMotor(), prefix,"RightMotor",motorPidDefaults);

            leftMotor.follow(rightMotor, contract.getLowerArmLeftMotor().inverted);

            configureCommonMotorProperties();
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
    public void setAbsoluteEncoderOffsetInDegrees(double offset) {
        absoluteEncoderOffsetInDegreesProp.set(offset);
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
        return 1.0 * Math.cos(getArmPositionInRadians());
    }

    @Override
    public void periodic() {
        super.periodic();
        if (isMotorReady()) {
            rightMotor.periodic();
        }
    }
}