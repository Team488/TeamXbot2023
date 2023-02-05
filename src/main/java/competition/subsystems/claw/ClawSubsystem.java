package competition.subsystems.claw;

import competition.electrical_contract.ElectricalContract;
import xbot.common.command.BaseSubsystem;
import xbot.common.controls.actuators.XSolenoid;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public class ClawSubsystem extends BaseSubsystem {
    public XSolenoid clawSolenoid;

    public enum ClawState {
        CloseCLaw,
        OpenClaw
    }

    @Inject
    public ClawSubsystem(XSolenoid.XSolenoidFactory solenoidFactory, ElectricalContract eContract, PropertyFactory propFactory) {
        this.clawSolenoid = solenoidFactory.create(eContract.getClawSolenoid().channel);
        clawSolenoid.setInverted(eContract.getClawSolenoid().inverted);
    }

    /**
     * Changes the state of the claw. True is considered "open".
     * @param state True opens the clow, false closes the claw
     */
    private void changeClaw(ClawState state) {
        if (state == ClawState.OpenClaw) {
            //open claw
            clawSolenoid.setOn(true);

        } else if (state == ClawState.CloseCLaw) {
            //close claw
            clawSolenoid.setOn(false);
        }
    }

    public boolean getClawState() {
        return clawSolenoid.getAdjusted();
    }

    public void open() {
        changeClaw(ClawState.OpenClaw);
    }

    public void close() {
        changeClaw(ClawState.CloseCLaw);
    }
}