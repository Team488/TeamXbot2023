package competition.subsystems.lights;

import javax.inject.Inject;
import javax.inject.Singleton;

import competition.electrical_contract.ElectricalContract;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import xbot.common.command.BaseSubsystem;
import xbot.common.controls.actuators.XDigitalOutput;
import xbot.common.controls.actuators.XPWM;
import xbot.common.controls.actuators.XDigitalOutput.XDigitalOutputFactory;
import xbot.common.controls.actuators.XPWM.XPWMFactory;
import xbot.common.properties.BooleanProperty;
import xbot.common.properties.PropertyFactory;
import xbot.common.properties.StringProperty;

@Singleton
public class LightsCommunicationSubsystem extends BaseSubsystem {

    XDigitalOutput dio0;
    XDigitalOutput dio1;
    XDigitalOutput dio2;
    XDigitalOutput dio3;
    XDigitalOutput allianceDio;
    XPWM analogOutput;

    XDigitalOutput[] dioOutputs;

    private int loopCounter;
    private final int loopMod = 5;

    private final StringProperty chosenState;
    private final BooleanProperty dio0Property;
    private final BooleanProperty dio1Property;
    private final BooleanProperty dio2Property;
    private final BooleanProperty dio3Property;
    private final BooleanProperty allianceDioProperty;

    public enum ArduinoStateMessage {
        RobotNotBooted(0),
        RobotDisabled(1),
        RobotEnabled(2);

        private int value;

        private ArduinoStateMessage(final int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }

    @Inject
    public LightsCommunicationSubsystem(XDigitalOutputFactory digitalOutputFactory, XPWMFactory pwmFactory,
            ElectricalContract contract, PropertyFactory pf) {

        dio0 = digitalOutputFactory.create(contract.getArduinoDio0().channel);
        dio1 = digitalOutputFactory.create(contract.getArduinoDio1().channel);
        dio2 = digitalOutputFactory.create(contract.getArduinoDio2().channel);
        dio3 = digitalOutputFactory.create(contract.getArduinoDio3().channel);
        allianceDio = digitalOutputFactory.create(contract.getArduinoAllianceDio().channel);

        dioOutputs = new XDigitalOutput[] { dio3, dio2, dio1, dio0 };
        loopCounter = 0;

        pf.setPrefix(this);
        chosenState = pf.createEphemeralProperty("ArduinoState", "Nothing Yet Set");
        dio0Property = pf.createEphemeralProperty("DIO0", false);
        dio1Property = pf.createEphemeralProperty("DIO1", false);
        dio2Property = pf.createEphemeralProperty("DIO2", false);
        dio3Property = pf.createEphemeralProperty("DIO3", false);
        allianceDioProperty = pf.createEphemeralProperty("AllianceDIO", false);

        this.register();
    }

    int counter = 0;

    @Override
    public void periodic() {
        // We don't need to run this every cycle.
        if (this.loopCounter++ % loopMod != 0) {
            return;
        }

        boolean dsEnabled = DriverStation.isEnabled();
        boolean isRedAlliance = DriverStation.getAlliance() == Alliance.Red;

        // Red alliance is 1, Blue alliance is 0.
        allianceDio.set(!isRedAlliance);

        ArduinoStateMessage currentState = ArduinoStateMessage.RobotNotBooted;

        // Figure out what we want to send to the arduino
        if (!dsEnabled) {
            currentState = ArduinoStateMessage.RobotDisabled;
        } else if (dsEnabled) {
            currentState = ArduinoStateMessage.RobotEnabled;
        }

        // Go through the bits of the state and set the appropriate dio
        // We start at the right and work our way left, so we start at
        // index 3 (picking up that leftmost 1). The dioOutputs array is ordered
        // such that the element at index 3 is dio0 to match how the arduino currently
        // parses the binary data.
        int stateValue = currentState.getValue();
        for (int i = 3; i >= 0; i--) {
            dioOutputs[3 - i].set(!((stateValue & (1 << i)) != 0));
        }

        chosenState.set(currentState.toString());
        dio0Property.set(dio0.get());
        dio1Property.set(dio1.get());
        dio2Property.set(dio2.get());
        dio3Property.set(dio3.get());
        allianceDioProperty.set(allianceDio.get());
    }
}