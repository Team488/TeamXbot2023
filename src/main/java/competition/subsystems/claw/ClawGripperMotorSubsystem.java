package competition.subsystems.claw;

import competition.electrical_contract.ElectricalContract;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import xbot.common.command.BaseSubsystem;
import xbot.common.command.NamedInstantCommand;
import xbot.common.command.NamedRunCommand;
import xbot.common.controls.actuators.XCANSparkMax;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;

public class ClawGripperMotorSubsystem extends BaseSubsystem {
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
            leaderMotor = sparkMaxFactory.create(eContract.getRightClawMotor(), getPrefix(), "Leader claw motor");
            followerMotor = sparkMaxFactory.create(eContract.getLeftClawMotor(), getPrefix(), "Follower claw motor");

            leaderMotor.setSmartCurrentLimit(5);
            followerMotor.setSmartCurrentLimit(5);

            followerMotor.follow(leaderMotor);
        }
    }

    public InstantCommand createStopCommand() {
        return new NamedInstantCommand("Stop claw motors", () -> {
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
}
