package competition.subsystems.arm;

import javax.inject.Inject;
import javax.inject.Singleton;

import com.fasterxml.jackson.databind.ser.Serializers;
import com.revrobotics.CANSparkMax;
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
    public ElectricalContract contract;
    public DoubleProperty powerProp;
    public DoubleProperty upperLimit;
    public  DoubleProperty lowerLimit;
    @Inject
    public  UpperArmSubsystem(XCANSparkMaxFactory sparkMaxFactory,ElectricalContract eContract, PropertyFactory propFactory){
        this.contract = eContract;
        if(contract.isUpperArmReady()){
            this.upperArmLeftMotor = sparkMaxFactory.create(eContract.getUpperArmLeftMotor(),this.getPrefix(),"upperArmLeftMotor");
            this.upperArmRightMotor = sparkMaxFactory.create(eContract.getUpperArmRightMotor(),this.getPrefix(),"upperArmRightMotor");
        }
        propFactory.setPrefix(this.getPrefix());
        this.powerProp = propFactory.createPersistentProperty("Standard Motor Power",1);
        upperLimit = propFactory.createPersistentProperty("upperLimit",0);
        lowerLimit = propFactory.createPersistentProperty("lowerLimit",0);
        setSoftLimit(false);
    }
    public void setMotorPower(double power){
        if(contract.isUpperArmReady()){
            upperArmLeftMotor.set(power);
            upperArmRightMotor.set(power);
        }
    }

    //set limits for arm rotation
    private void setSoftLimit(boolean enabled){
        if(contract.isUpperArmReady()){
            if(enabled){
                enableSoftLimit(true);
                configSoftLimit(upperLimit.get(),lowerLimit.get());

            }
                enableSoftLimit(false);
        }
    }
    private void enableSoftLimit(boolean on){
        upperArmLeftMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward,on);
        upperArmRightMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward,on);
        upperArmLeftMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,on);
        upperArmRightMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,on);
    }
    private void configSoftLimit(double upper, double lower){
        upperArmLeftMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward,(float)upper);
        upperArmRightMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward,(float)upper);
        upperArmLeftMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,(float)lower);
        upperArmRightMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,(float)lower);
    }


    public void forwardRotation(){
        setMotorPower(powerProp.get());
    }
    public void backwardRotation(){
        setMotorPower(powerProp.get());
    }
}
