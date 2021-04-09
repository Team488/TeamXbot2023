
package competition;

import competition.injection.CompetitionModule;
import competition.injection.SimulationModule;
import competition.operator_interface.OperatorCommandMap;
import competition.subsystems.SubsystemDefaultCommandMap;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import xbot.common.command.BaseRobot;
import xbot.common.math.FieldPose;
import xbot.common.subsystems.pose.BasePoseSubsystem;

public class Robot extends BaseRobot {

    @Override
    protected void initializeSystems() {
        super.initializeSystems();
        this.injector.getInstance(SubsystemDefaultCommandMap.class);
        this.injector.getInstance(OperatorCommandMap.class);
    }

    @Override
    protected void setupInjectionModule() {
        if (BaseRobot.isReal()) {
            this.injectionModule = new CompetitionModule(true);
        } else {
            this.injectionModule = new SimulationModule();
        }
    }

    @Override
    public void simulationInit() {
        super.simulationInit();
        // Automatically enables the robot; remove this line of code if you want the robot
        // to start in a disabled state (as it would on the field). However, this does save you the 
        // hassle of navigating to the DS window and re-enabling the simulated robot.
        DriverStationSim.setEnabled(true);
        webots.setFieldPoseOffset(getFieldOrigin());
    }

    private FieldPose getFieldOrigin() {
        // Modify this to whatever the simulator coordinates are for the "FRC origin" of the field.
        // From a birds-eye view where your alliance station is at the bottom, this is the bottom-left corner
        // of the field.
        return new FieldPose(
            -2.33*PoseSubsystem.INCHES_IN_A_METER, 
            -4.58*PoseSubsystem.INCHES_IN_A_METER, 
            BasePoseSubsystem.FACING_TOWARDS_DRIVERS
            );
    }
}
