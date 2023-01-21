package competition.subsystems.arm;

import javax.inject.Inject;
import javax.inject.Singleton;

import competition.electrical_contract.ElectricalContract;
import xbot.common.controls.actuators.XCANSparkMax;
import xbot.common.controls.actuators.XCANSparkMax.XCANSparkMaxFactory;
import xbot.common.math.PIDManager;
import xbot.common.math.XYPair;
import xbot.common.properties.BooleanProperty;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;
import xbot.common.properties.StringProperty;
import xbot.common.command.BaseSubsystem;

@Singleton
public class LowerArmSubsystem extends BaseSubsystem{
    public XCANSparkMax lowerArmLeftMotor;
    public XCANSparkMax lowerArmRightMotor;
    public DoubleProperty powerProp;

    public final DoubleProperty extendLimit;
    public final DoubleProperty retractLimit;

    @Inject
    public LowerArmSubsystem(XCANSparkMaxFactory sparkMaxFactory, ElectricalContract eContract, PropertyFactory propFactory){
        this.lowerArmLeftMotor = sparkMaxFactory.create(eContract.getLowerArmLeftMotor(),this.getPrefix(),"lowerArmLeftMotor");
        this.lowerArmRightMotor = sparkMaxFactory.create(eContract.getLowerArmRightMotor(),this.getPrefix(),"lowerArmRightMotor");
        propFactory.setPrefix(this.getPrefix());

        this.powerProp = propFactory.createPersistentProperty("Standard Motor Power", 1);
        extendLimit = propFactory.createPersistentProperty("extendLimit",0);
        retractLimit = propFactory.createPersistentProperty("retractLimit",0);
    }

    private void setMotorPower(double power){
        lowerArmLeftMotor.set(power);
        lowerArmRightMotor.set(power);
    }
    //set limits for extending and retracting
    public void configSoftLimits(){
    }
    private void enableSoftLimits(){

    }
    private void disableSoftLimits(){

    }
    public void goForward(){
        setMotorPower(powerProp.get());
    }

    public void goBack(){
        setMotorPower(-powerProp.get());
    }
}