package competition.subsystems.arm;

import javax.inject.Inject;
import javax.inject.Singleton;

import com.revrobotics.CANSparkMax;
import competition.electrical_contract.ElectricalContract;
import xbot.common.command.BaseSetpointSubsystem;
import xbot.common.controls.actuators.XCANSparkMax;
import xbot.common.controls.actuators.XCANSparkMax.XCANSparkMaxFactory;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

@Singleton
public class UpperArmSubsystem extends BaseSetpointSubsystem {
    public XCANSparkMax upperArmLeftMotor;
    public XCANSparkMax upperArmRightMotor;
    public ElectricalContract contract;
    public DoubleProperty powerProp;
    public DoubleProperty upperLimit;
    public  DoubleProperty lowerLimit;

    public final DoubleProperty positionLowerGoalProperty;
    public final DoubleProperty positionMidGoalProperty;
    public final DoubleProperty positionHighGoalProperty;
    public final DoubleProperty positionFullyRetractedProperty;

    private  double goal;

    private enum PidSlot{
        Position(0),
        Velocity(1);
        private final int slot;
        private PidSlot(int slot){
            this.slot = slot;
        }
        public int getSlot(){
            return slot;
        }

    }
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

        positionLowerGoalProperty = propFactory.createPersistentProperty("LowerGoalPositionInches",0);
        positionMidGoalProperty = propFactory.createPersistentProperty("MidGoalPositionInches",0);
        positionHighGoalProperty = propFactory.createPersistentProperty("HighGoalPositionInches",0);
        positionFullyRetractedProperty = propFactory.createPersistentProperty("FullRetractedPositionInches",0);

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

    @Override
    public double getCurrentValue() {
        return 0;
    }

    @Override
    public double getTargetValue() {
        return goal;
    }

    @Override
    public void setTargetValue(double value) {
        goal = value;
    }

    @Override
    public void setPower(double power) {

    }

    @Override
    public boolean isCalibrated() {
        return false;
    }
}
