package competition.subsystems.collector;

import competition.electrical_contract.ElectricalContract;
import competition.operator_interface.OperatorInterface;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.Logger;
import xbot.common.advantage.DataFrameRefreshable;
import xbot.common.command.BaseSubsystem;
import xbot.common.controls.actuators.XCANSparkMax;
import xbot.common.controls.actuators.XSolenoid;
import xbot.common.controls.sensors.XAnalogInput;
import xbot.common.controls.sensors.XTimer;
import xbot.common.properties.BooleanProperty;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public class CollectorSubsystem extends BaseSubsystem implements DataFrameRefreshable {
    public XCANSparkMax collectorMotor;
    public XSolenoid collectorSolenoid;
    public DoubleProperty intakePower;
    public DoubleProperty ejectPower;
    private CollectorState currentState;
    final ElectricalContract contract;
    public double intakeTime;
    public double currentIntakeTime;
    private boolean gamePieceCollected;
    boolean intake = false;
    private int loopCount;

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
            collectorMotor.setSmartCurrentLimit(5);
            pressureSensor = analogInputFactory.create(eContract.getPressureSensor().channel);
        }
        pf.setPrefix(this);
        intakePower = pf.createPersistentProperty("intakePower", 1);
        ejectPower = pf.createPersistentProperty("retractPower", -1);
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
            intakeTime = XTimer.getFPGATimestamp();
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
    public void refreshDataFrame() {
        if (contract.isCollectorReady()) {
            collectorMotor.refreshDataFrame();
        }
    }

    @Override
    public void periodic() {
        if (contract.isCollectorReady()) {

            currentIntakeTime = XTimer.getFPGATimestamp();
            if ((currentIntakeTime - intakeTime > 0.5) && intake) {
                //check current RPM is less than 500
                gamePieceCollected = collectorMotor.getVelocity() < 500;
            }else{
                gamePieceCollected = false;
            }
            //if game piece is collected, rumble controller

            double intensity = 0.5;
            if (currentState == CollectorState.Retracted) {
                intensity = 0.1;
            }

            if(getGamePieceCollected()){
                oi.operatorGamepad.getRumbleManager().rumbleGamepad(intensity,0.1);
            }else{
                oi.operatorGamepad.getRumbleManager().stopGamepadRumble();
            }
        }

        Logger.getInstance().recordOutput(this.getPrefix() + "CollectorState", currentState.toString());
        Logger.getInstance().recordOutput(this.getPrefix() + "GamePieceCollected", gamePieceCollected);
        Logger.getInstance().recordOutput(this.getPrefix() + "Pressure", pressureSensor.getVoltage());
    }

    public boolean getGamePieceCollected() {
        return gamePieceCollected;
    }
}
