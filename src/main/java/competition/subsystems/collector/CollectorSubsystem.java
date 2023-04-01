package competition.subsystems.collector;

import competition.electrical_contract.ElectricalContract;
import competition.operator_interface.OperatorInterface;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import xbot.common.command.BaseSubsystem;
import xbot.common.controls.actuators.XCANSparkMax;
import xbot.common.controls.actuators.XSolenoid;
import xbot.common.controls.sensors.XAnalogInput;
import xbot.common.controls.sensors.XTimer;
import xbot.common.logic.TimeStableValidator;
import xbot.common.properties.BooleanProperty;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.Property;
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
    public final DoubleProperty currentMotorVelocity;
    public DoubleProperty intakeTime;
    public DoubleProperty currentIntakeTime;
    private final BooleanProperty gamePieceCollected;
    boolean intake = false;
    private int loopCount;

    private TimeStableValidator gamePieceCollectedValidator;
    private boolean gamePieceCollectedAndStable = false;

    final OperatorInterface oi;
    private XAnalogInput pressureSensor;

    public enum CollectorState {
        Extended,
        Retracted
    }

    @Inject
    public CollectorSubsystem(XCANSparkMax.XCANSparkMaxFactory sparkMaxFactory, PropertyFactory pf,
                              XSolenoid.XSolenoidFactory xSolenoidFactory, XAnalogInput.XAnalogInputFactory analogInputFactory,
                              ElectricalContract eContract, OperatorInterface oi) {
        this.contract = eContract;
        this.oi = oi;
        this.currentState = CollectorState.Retracted;
        if (contract.isCollectorReady()) {
            this.collectorMotor = sparkMaxFactory.createWithoutProperties(eContract.getCollectorMotor(), getPrefix(), "CollectorMotor");
            this.collectorSolenoid = xSolenoidFactory.create(eContract.getCollectorSolenoid().channel);
            collectorMotor.setSmartCurrentLimit(7);
            pressureSensor = analogInputFactory.create(eContract.getPressureSensor().channel);
        }
        pf.setPrefix(this);
        pf.setDefaultLevel(Property.PropertyLevel.Debug);
        intakePower = pf.createPersistentProperty("intakePower", 1);
        ejectPower = pf.createPersistentProperty("retractPower", -0.25);

        pf.setDefaultLevel(Property.PropertyLevel.Debug);
        currentIntakeTime = pf.createEphemeralProperty("currentIntakeTime", 0);
        intakeTime = pf.createEphemeralProperty("intakeTime", 0);
        currentMotorVelocity = pf.createEphemeralProperty("currentMotorVelocity", 0);
        gamePieceCollected = pf.createEphemeralProperty("gamePieceCollected", false);

        gamePieceCollectedValidator = new TimeStableValidator(0.75);
        gamePieceCollectedValidator.checkStable(false);
    }

    private void changeCollector(CollectorState state) {
        currentState = state;
        if (state == CollectorState.Extended) {
            collectorSolenoid.setOn(true);
        } else if (state == CollectorState.Retracted) {
            collectorSolenoid.setOn(false);
        }
    }

    public CollectorState getCollectorState() {
        return currentState;
    }

    public void extend() {
        changeCollector(CollectorState.Extended);
    }

    public void retract() {
        changeCollector(CollectorState.Retracted);
    }

    private void setMotorPower(double power) {
        if (contract.isCollectorReady()) {
            collectorMotor.set(power);
        }
    }

    public void intake() {
        setMotorPower(intakePower.get());
        if (!intake) {
            intakeTime.set(XTimer.getFPGATimestamp());
        }
        intake = true;
    }

    public void eject() {
        setMotorPower(ejectPower.get());
        intake = false;
    }

    public void stop() {
        setMotorPower(0);
        intake = false;
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

    public Command getCollectThenAutomaticallyRetractCommand() {
        return new FunctionalCommand(
                () -> {log.info("Initializing");},
                () -> {
                    this.intake();
                    if (this.gamePieceCollectedAndStable) {
                        this.retract();
                    } else {
                        this.extend();
                    }
                },
                (end) -> {this.retract();},
                () -> {return false;},
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

    @Override
    public void periodic() {
        loopCount++;
        if (contract.isCollectorReady()) {
            if (loopCount % 250 == 0) {
                log.info("PressureSensorValue:" + pressureSensor.getVoltage());
            }
            currentIntakeTime.set(XTimer.getFPGATimestamp());
            if ((currentIntakeTime.get() - intakeTime.get() > 0.5) && intake) {
                //check current RPM is less than 500
                currentMotorVelocity.set(collectorMotor.getVelocity());
                boolean collectorStalled = currentMotorVelocity.get() < 500;
                gamePieceCollected.set(collectorStalled);
                gamePieceCollectedAndStable = gamePieceCollectedValidator.checkStable(collectorStalled);
            }else{
                gamePieceCollected.set(false);
                gamePieceCollectedAndStable = gamePieceCollectedValidator.checkStable(false);
            }
            //if game piece is collected, rumble controller

            double intensity = 0.5;
            if (currentState == CollectorState.Retracted) {
                intensity = 0.1;
            }

            if(getGamePieceCollected()){
                oi.operatorGamepad.getRumbleManager().rumbleGamepad(intensity,0.1);
                oi.driverGamepad.getRumbleManager().rumbleGamepad(intensity,0.1);
            }else{
                oi.operatorGamepad.getRumbleManager().stopGamepadRumble();
                oi.driverGamepad.getRumbleManager().stopGamepadRumble();
            }
        }
    }

    public boolean getGamePieceCollected() {
        return gamePieceCollected.get();
    }
}
