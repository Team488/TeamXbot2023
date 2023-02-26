package competition.operator_interface;

import javax.inject.Inject;
import javax.inject.Singleton;

import competition.subsystems.drive.commands.AutoBalanceCommand;
import xbot.common.controls.sensors.XXboxController;
import xbot.common.controls.sensors.XXboxController.XXboxControllerFactory;
import xbot.common.controls.sensors.XXboxController.XboxButton;
import xbot.common.logging.RobotAssertionManager;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

/**
 * This class is the glue that binds the controls on the physical operator interface to the commands and command groups
 * that allow control of the robot.
 */
@Singleton
public class OperatorInterface {
    public XXboxController driverGamepad;
    public XXboxController operatorGamepad;
    public XXboxController experimentalGamepad;
    
    final DoubleProperty driverDeadband;
    final DoubleProperty operatorDeadband;

    @Inject
    public OperatorInterface(XXboxControllerFactory controllerFactory, RobotAssertionManager assertionManager, PropertyFactory pf) {
        driverGamepad = controllerFactory.create(0);
        driverGamepad.setLeftInversion(false, true);
        driverGamepad.setRightInversion(true, true);

        operatorGamepad = controllerFactory.create(1);
        operatorGamepad.setLeftInversion(false, true);
        operatorGamepad.setRightInversion(false, true);

        experimentalGamepad = controllerFactory.create(2);
        experimentalGamepad.setLeftInversion(false, true);
        experimentalGamepad.setRightInversion(false, true);

        pf.setPrefix("OperatorInterface");
        driverDeadband = pf.createPersistentProperty("Driver Deadband", 0.12);
        operatorDeadband = pf.createPersistentProperty("Operator Deadband", 0.15);
    }

    public double getDriverGamepadTypicalDeadband() {
        return driverDeadband.get();
    }

    public double getOperatorGamepadTypicalDeadband() {
        return operatorDeadband.get();
    }

}
