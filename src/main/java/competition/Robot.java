
package competition;

import competition.injection.components.BaseRobotComponent;
import competition.injection.components.DaggerPracticeComponent;
import competition.injection.components.DaggerRobotComponent;
import competition.injection.components.DaggerSimulationComponent;
import competition.injection.components.RobotComponent;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import xbot.common.command.BaseRobot;
import xbot.common.math.FieldPose;
import xbot.common.subsystems.pose.BasePoseSubsystem;

public class Robot extends BaseRobot {

    @Override
    protected void initializeSystems() {
        super.initializeSystems();
        getInjectorComponent().subsystemDefaultCommandMap();
        getInjectorComponent().operatorCommandMap();

        dataFrameRefreshables.add((DriveSubsystem)getInjectorComponent().driveSubsystem());
        dataFrameRefreshables.add(getInjectorComponent().poseSubsystem());
        dataFrameRefreshables.add(getInjectorComponent().unifiedArmSubsystem());
    }

    protected BaseRobotComponent createDaggerComponent() {
        if (BaseRobot.isReal()) {
            // TODO: Figure out some elegant way to detect 2022 vs 2023 chassis and return the appropriate component.
            // until then, you'll have to manually change this line to return the correct component.
            return DaggerPracticeComponent.create();
            //return DaggerRobotComponent.create();

        } else {
            return DaggerSimulationComponent.create();
        }
    }

    public BaseRobotComponent getInjectorComponent() {
        return (BaseRobotComponent)super.getInjectorComponent();
    }

    @Override
    public void disabledInit() {
        super.disabledInit();
        ((PoseSubsystem)getInjectorComponent().poseSubsystem()).updateAllianceFromDriverStation();
    }

    @Override
    public void autonomousInit() {
        super.autonomousInit();
        ((PoseSubsystem)getInjectorComponent().poseSubsystem()).updateAllianceFromDriverStation();
    }

    @Override
    public void teleopInit() {
        super.teleopInit();
        ((PoseSubsystem)getInjectorComponent().poseSubsystem()).updateAllianceFromDriverStation();
    }

    @Override
    public void simulationInit() {
        super.simulationInit();
        ((PoseSubsystem)getInjectorComponent().poseSubsystem()).updateAllianceFromDriverStation();
        // Automatically enables the robot; remove this line of code if you want the robot
        // to start in a disabled state (as it would on the field). However, this does save you the 
        // hassle of navigating to the DS window and re-enabling the simulated robot.
        //DriverStationSim.setEnabled(true);
        //webots.setFieldPoseOffset(getFieldOrigin());
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
