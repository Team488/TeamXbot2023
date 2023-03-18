package competition.subsystems.claw;

import competition.electrical_contract.ElectricalContract;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import xbot.common.advantage.DataFrameRefreshable;
import xbot.common.command.BaseSubsystem;
import xbot.common.command.NamedRunCommand;
import xbot.common.controls.actuators.XCANSparkMax;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public class ClawGripperMotorSubsystem extends BaseSubsystem implements DataFrameRefreshable {
    private final ElectricalContract electricalContract;

    private final DoubleProperty intakePower;
    private final DoubleProperty intakeDuration;

    private XCANSparkMax leaderMotor;
    private XCANSparkMax followerMotor;

    @Inject
    public ClawGripperMotorSubsystem(PropertyFactory pf, XCANSparkMax.XCANSparkMaxFactory sparkMaxFactory, ElectricalContract eContract) {
        pf.setPrefix(this);

        intakePower = pf.createPersistentProperty("Intake power", 0.2);
        intakeDuration = pf.createPersistentProperty("Intake duration", 1);

        electricalContract = eContract;

        if (eContract.areClawMotorsReady()) {
            leaderMotor = sparkMaxFactory.createWithoutProperties(eContract.getRightClawMotor(), getPrefix(), "Leader claw motor");
            followerMotor = sparkMaxFactory.createWithoutProperties(eContract.getLeftClawMotor(), getPrefix(), "Follower claw motor");

            leaderMotor.setSmartCurrentLimit(40);
            followerMotor.setSmartCurrentLimit(40);

            followerMotor.follow(leaderMotor, eContract.getLeftClawMotor().inverted);
        }
    }

    public Command createStopCommand() {
        return new NamedRunCommand("Stop claw motors", () -> {
            this.setStopped();
        }, this);
    }

    public Command createIntakeCommand() {
        return new NamedRunCommand("Claw motors to intake", () -> {
            this.setIntake();
        }, this);
    }

    public Command createIntakeBurstCommand() {
        return createIntakeCommand().withTimeout(intakeDuration.get());
    }

    public void setStopped() {
        if (electricalContract.areClawMotorsReady()) {
            leaderMotor.stopMotor();
        }
    }

    public void setIntake() {
        if (electricalContract.areClawMotorsReady()) {
            leaderMotor.set(intakePower.get());
        }
    }

    public NamedRunCommand setEject(double eject){
        if(electricalContract.areClawMotorsReady()){
            return new NamedRunCommand("Claw Eject", () -> {
                leaderMotor.set(eject);
            });

        }
        return null;
    }

    @Override
    public void refreshDataFrame() {
        if (electricalContract.areClawMotorsReady()) {
            leaderMotor.refreshDataFrame();
            followerMotor.refreshDataFrame();
        }
    }
}
