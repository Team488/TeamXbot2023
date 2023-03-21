package competition.subsystems.lights;

import javax.inject.Inject;
import javax.inject.Singleton;

import competition.electrical_contract.ElectricalContract;
import competition.subsystems.arm.UnifiedArmSubsystem;
import competition.subsystems.collector.CollectorSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import xbot.common.command.BaseSubsystem;
import xbot.common.controls.actuators.XDigitalOutput;
import xbot.common.controls.actuators.XDigitalOutput.XDigitalOutputFactory;
import xbot.common.controls.actuators.XPWM.XPWMFactory;
import xbot.common.properties.BooleanProperty;
import xbot.common.properties.PropertyFactory;
import xbot.common.properties.StringProperty;

@Singleton
public class LightsCommunicationSubsystem extends BaseSubsystem {

    final XDigitalOutput dio0;
    final XDigitalOutput dio1;
    final XDigitalOutput dio2;
    final XDigitalOutput dio3;
    final XDigitalOutput dio4;
    final XDigitalOutput cubeDio;

    final XDigitalOutput[] dioOutputs;

    private int loopCounter;
    private final int loopMod = 5;

    private final StringProperty chosenState;
    private final BooleanProperty dio0Property;
    private final BooleanProperty dio1Property;
    private final BooleanProperty dio2Property;
    private final BooleanProperty dio3Property;
    private final BooleanProperty dio4Property;
    private final BooleanProperty cubeDioProperty;

    private final CollectorSubsystem collector;
    private final UnifiedArmSubsystem arm;

    public enum LightsStateMessage {
        // when no code is running, all the dios are high by default so the max value is what's sent
        RobotNotBooted(31),
        RobotDisabled(1),
        Enabled(2),
        GamePieceCollected(3);

        private int value;

        private LightsStateMessage(final int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }

    @Inject
    public LightsCommunicationSubsystem(XDigitalOutputFactory digitalOutputFactory, XPWMFactory pwmFactory,
            ElectricalContract contract, PropertyFactory pf, CollectorSubsystem collector, UnifiedArmSubsystem arm) {

        dio0 = digitalOutputFactory.create(contract.getLightsDio0().channel);
        dio1 = digitalOutputFactory.create(contract.getLightsDio1().channel);
        dio2 = digitalOutputFactory.create(contract.getLightsDio2().channel);
        dio3 = digitalOutputFactory.create(contract.getLightsDio3().channel);
        dio4 = digitalOutputFactory.create(contract.getLightsDio4().channel);
        cubeDio = digitalOutputFactory.create(contract.getLightsCubeDio().channel);

        dioOutputs = new XDigitalOutput[] { dio0, dio1, dio2, dio3, dio4 };
        loopCounter = 0;

        pf.setPrefix(this);
        chosenState = pf.createEphemeralProperty("ArduinoState", "Nothing Yet Set");
        dio0Property = pf.createEphemeralProperty("DIO0", false);
        dio1Property = pf.createEphemeralProperty("DIO1", false);
        dio2Property = pf.createEphemeralProperty("DIO2", false);
        dio3Property = pf.createEphemeralProperty("DIO3", false);
        dio4Property = pf.createEphemeralProperty("DIO4", false);
        cubeDioProperty = pf.createEphemeralProperty("IsConeDIO", false);

        this.collector = collector;
        this.arm = arm;

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

        // Red alliance is 1, Blue alliance is 0.
        boolean isCube = arm.isCubeMode();
        cubeDio.set(isCube);

        LightsStateMessage currentState = LightsStateMessage.RobotNotBooted;


        // Figure out what we want to send to the arduino
        if (!dsEnabled) {
            currentState = LightsStateMessage.RobotDisabled;
        } else if (collector.getGamePieceCollected()) {
            currentState = LightsStateMessage.GamePieceCollected;
        } else if (dsEnabled) {
            currentState = LightsStateMessage.Enabled;
        }

        int stateValue = currentState.getValue();
        for (int i = 0; i < dioOutputs.length; i++) {
            dioOutputs[i].set(((stateValue & (1 << i)) != 0));
        }

        chosenState.set(currentState.toString());
        dio0Property.set(dio0.get());
        dio1Property.set(dio1.get());
        dio2Property.set(dio2.get());
        dio3Property.set(dio3.get());
        dio4Property.set(dio4.get());
        cubeDioProperty.set(cubeDio.get());
    }
}