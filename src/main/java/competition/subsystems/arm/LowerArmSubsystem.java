package competition.subsystems.arm;

import javax.inject.Inject;
import javax.inject.Singleton;

import com.revrobotics.CANSparkMax;
import competition.electrical_contract.ElectricalContract;
import xbot.common.command.BaseSetpointSubsystem;
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
public class LowerArmSubsystem extends BaseSetpointSubsystem {
    public XCANSparkMax lowerArmLeftMotor;
    public XCANSparkMax lowerArmRightMotor;
    public ElectricalContract contract;
    public DoubleProperty powerProp;

    public  DoubleProperty extendLimit;
    public  DoubleProperty retractLimit;
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

        public  int getSlot(){
            return slot;
        }
    }
    @Inject
    public LowerArmSubsystem(XCANSparkMaxFactory sparkMaxFactory, ElectricalContract eContract, PropertyFactory propFactory){
        this.contract = eContract;
        if(contract.isLowerArmReady()){
            this.lowerArmLeftMotor = sparkMaxFactory.create(eContract.getLowerArmLeftMotor(),this.getPrefix(),"lowerArmLeftMotor");
            this.lowerArmRightMotor = sparkMaxFactory.create(eContract.getLowerArmRightMotor(),this.getPrefix(),"lowerArmRightMotor");
        }
        propFactory.setPrefix(this.getPrefix());
        this.powerProp = propFactory.createPersistentProperty("Standard Motor Power", 1);
        extendLimit = propFactory.createPersistentProperty("extendLimit",0);
        retractLimit = propFactory.createPersistentProperty("retractLimit",0);
        setSoftLimit(false);

        positionLowerGoalProperty = propFactory.createPersistentProperty("LowerGoalPositionInches",0);
        positionMidGoalProperty = propFactory.createPersistentProperty("MidGoalPositionInches",0);
        positionHighGoalProperty = propFactory.createPersistentProperty("HighGoalPositionInches",0);
        positionFullyRetractedProperty = propFactory.createPersistentProperty("FullRetractedPositionInches",0);
    }

    public void setMotorPower(double power){
        if(contract.isLowerArmReady()){
            lowerArmLeftMotor.set(power);
            lowerArmRightMotor.set(power);
        }
    }

    //set limits for extending and retracting
    private void setSoftLimit(boolean enabled){
        if(contract.isLowerArmReady()){
            if(enabled){
                enableSoftLimit(true);
                configSoftLimit(extendLimit.get(),retractLimit.get());

            }
                enableSoftLimit(false);
        }
    }
    private void configSoftLimit(double extend,double retract){
        lowerArmLeftMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward,(float)extend);
        lowerArmRightMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward,(float)extend);
        lowerArmLeftMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,(float)retract);
        lowerArmRightMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,(float)retract);
    }
    private void enableSoftLimit(boolean on){
        lowerArmLeftMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward,on);
        lowerArmRightMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward,on);
        lowerArmLeftMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,on);
        lowerArmRightMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,on);
    }

    public void goForward(){
        setMotorPower(powerProp.get());
    }

    public void goBack(){
        setMotorPower(-powerProp.get());
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