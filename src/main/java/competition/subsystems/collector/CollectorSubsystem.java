package competition.subsystems.collector;

import competition.electrical_contract.ElectricalContract;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
    private CollectorState currentState;
    final ElectricalContract contract;

    public enum CollectorState{
        Extended,
        Retracted
    }

    public enum CollectionState {
        Safe,
        Collecting,
        Holding,
        Ejecting
    }

    private CollectionState currentCollectionState = CollectionState.Safe;

    @Inject
    public CollectorSubsystem(XCANSparkMax.XCANSparkMaxFactory sparkMaxFactory, PropertyFactory pf,
                              XSolenoid.XSolenoidFactory xSolenoidFactory,
                              ElectricalContract eContract){
        this.contract = eContract;
        this.currentState = CollectorState.Retracted;
        if(contract.isCollectorReady()){
            this.collectorMotor = sparkMaxFactory.create(eContract.getCollectorMotor(),getPrefix(),"CollectorMotor");
            this.collectorSolenoid = xSolenoidFactory.create(eContract.getCollectorSolenoid().channel);
        }
        pf.setPrefix(this);
        intakePower = pf.createPersistentProperty("intakePower",1);
        ejectPower = pf.createPersistentProperty("retractPower", -1);

    }

    public void setCollectionState(CollectionState state) {
        currentCollectionState = state;
    }

    public CollectionState getCollectionState() {
        return currentCollectionState;
    }

    private void changeCollector(CollectorState state){
        currentState = state;
        if(state == CollectorState.Extended){
            collectorSolenoid.setOn(true);
        } else if (state == CollectorState.Retracted) {
            collectorSolenoid.setOn(false);
        }
    }
    public CollectorState getCollectorState(){
        return currentState;
    }

    public void extend(){
        changeCollector(CollectorState.Extended);
    }

    public void retract(){
        changeCollector(CollectorState.Retracted);
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

    public Command getCollectThenRetractCommand() {
        return Commands.startEnd(
                () -> {
                    log.info("Collecting");
                    this.intake();
                    this.extend();
                },
                () -> {
                    this.retract();
                },
                this);
    }

    public Command getEjectThenStopCommand() {
        return Commands.startEnd(
                () -> {
                    log.info("Ejecting");
                    this.retract();
                    this.eject();
                },
                () -> {
                    this.stop();
                },
                this);
    }
}
