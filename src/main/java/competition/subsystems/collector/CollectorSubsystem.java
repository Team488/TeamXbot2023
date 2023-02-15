package competition.subsystems.collector;

import competition.electrical_contract.ElectricalContract;
import xbot.common.command.BaseSubsystem;
import xbot.common.controls.actuators.XCANSparkMax;
import xbot.common.controls.actuators.XSolenoid;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public class CollectorSubsystem extends BaseSubsystem {
    public XCANSparkMax collectorMotor;
    public XSolenoid collectorSolenoid;
    public DoubleProperty intakePower;
    public DoubleProperty ejectPower;
    final ElectricalContract contract;

    public enum collectorState{
        extend,
        retract
    }

    @Inject
    public CollectorSubsystem(XCANSparkMax.XCANSparkMaxFactory sparkMaxFactory, PropertyFactory pf,
                              XSolenoid.XSolenoidFactory xSolenoidFactory,
                              ElectricalContract eContract){
        this.contract = eContract;
        if(contract.isCollectorReady()){
            this.collectorMotor = sparkMaxFactory.create(eContract.getCollectorMotor(),getPrefix(),"CollectorMotor");
            this.collectorSolenoid = xSolenoidFactory.create(eContract.getCollectorSolenoid().channel);
        }
        pf.setPrefix(this);
        intakePower = pf.createPersistentProperty("intakePower",1);
        ejectPower = pf.createPersistentProperty("retractPower", -1);

    }

    private void changeCollector(collectorState state){
        if(state == collectorState.extend){
            collectorSolenoid.setOn(true);
        } else if (state == collectorState.retract) {
            collectorSolenoid.setOn(false);
        }
    }
    public boolean getCollectorState(){
        return collectorSolenoid.getAdjusted();
    }

    public void extend(){
        changeCollector(collectorState.extend);
    }

    public void retract(){
        changeCollector(collectorState.retract);
    }

    private void setMotorPower(double power){
        if(contract.isCollectorReady()){
            collectorMotor.set(power);
        }
    }

    public void intake(){
        setMotorPower(intakePower.get());
    }
    public void eject(){
        setMotorPower(ejectPower.get());
    }

    public void stop(){
        setMotorPower(0);
    }


}
