package competition.injection.components;

import competition.injection.swerve.SwerveComponentHolder;
import competition.operator_interface.OperatorCommandMap;
import competition.operator_interface.OperatorInterface;
import competition.subsystems.SubsystemDefaultCommandMap;
import competition.subsystems.arm.UnifiedArmSubsystem;
import competition.subsystems.lights.LightsCommunicationSubsystem;
import xbot.common.injection.components.BaseComponent;

public abstract class BaseRobotComponent extends BaseComponent {

    public abstract SubsystemDefaultCommandMap subsystemDefaultCommandMap();

    public abstract OperatorCommandMap operatorCommandMap();

    public abstract OperatorInterface operatorInterface();

    public abstract SwerveComponentHolder swerveComponents();

    public abstract UnifiedArmSubsystem unifiedArmSubsystem();
    public abstract LightsCommunicationSubsystem lightsCommunicationSubsystem();
}
