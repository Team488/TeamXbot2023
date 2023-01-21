package competition.subsystems.arm;

import javax.inject.Inject;
import javax.inject.Singleton;

import com.fasterxml.jackson.databind.ser.Serializers;
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
public class UpperArmSubsystem extends BaseSubsystem {
    public XCANSparkMax upperArmLeftMotor;
    public XCANSparkMax upperArmRightMotor;
    public DoubleProperty powerProp;
    public DoubleProperty upperLimit;
    public  DoubleProperty lowerLimit;
    @Inject
    public  UpperArmSubsystem(XCANSparkMaxFactory sparkMaxFactory,ElectricalContract eContract, PropertyFactory propFactory){
        this.upperArmLeftMotor = sparkMaxFactory.create(eContract.getUpperArmLeftMotor(),this.getPrefix(),"upperArmLeftMotor");
        this.upperArmRightMotor = sparkMaxFactory.create(eContract.getUpperArmRightMotor(),this.getPrefix(),"upperArmRightMotor");
        propFactory.setPrefix(this.getPrefix());
        this.powerProp = propFactory.createPersistentProperty("Standard Motor Power",1);
        upperLimit = propFactory.createPersistentProperty("upperLimit",0);
        lowerLimit = propFactory.createPersistentProperty("lowerLimit",0);
    }
    private void setMotorPower(double power){
        upperArmLeftMotor.set(power);
        upperArmRightMotor.set(power);
    }
    //set limits for arm rotation
    public void configSoftLimits(){
        upperLimit = (int) upperArmMaxHeight.get();
        lowerLimit = (int) startingPos; //set lower limit to maximum backwards rotation
    }

    private void enableSoftLimits(){

    }
    private void disableSoftLimits(){

    }
    public void forwardRotation(){
        setMotorPower(powerProp.get());
    }
    public void backwardRotation(){
        setMotorPower(powerProp.get());
    }
}
