package competition.injection.components;

import competition.electrical_contract.ElectricalContract;
import competition.injection.modules.CommonModule;
import competition.injection.modules.CompetitionTestModule;
import competition.subsystems.arm.LowerArmSegment;
import competition.subsystems.arm.UnifiedArmSubsystem;
import competition.subsystems.arm.UpperArmSegment;
import competition.subsystems.arm.commands.ControlEndEffectorPositionCommand;
import competition.subsystems.arm.commands.SimpleSafeArmRouterCommand;
import competition.subsystems.arm.commands.SimpleXZRouterCommand;
import competition.subsystems.arm.commands.UnifiedArmMaintainer;
import competition.subsystems.claw.ClawSubsystem;
import competition.subsystems.claw.CloseClawCommand;
import competition.subsystems.claw.OpenClawCommand;
import competition.subsystems.arm.commands.ControlArmsWithJoyStickCommand;
import competition.subsystems.drive.commands.SetSwerveMotorControllerPidParametersCommand;
import competition.subsystems.drive.commands.SwerveDriveWithJoysticksCommand;
import competition.subsystems.drive.commands.SwerveToPointCommand;
import competition.subsystems.drive.swerve.SwerveSteeringMotorPidSubsystem;
import competition.subsystems.vision.VisionSubsystem;
import dagger.Component;
import xbot.common.injection.modules.MockControlsModule;
import xbot.common.injection.modules.MockDevicesModule;
import xbot.common.injection.modules.UnitTestModule;

import javax.inject.Singleton;

@Singleton
@Component(modules = { UnitTestModule.class, MockDevicesModule.class, MockControlsModule.class, CompetitionTestModule.class, CommonModule.class })
public abstract class CompetitionTestComponent extends BaseRobotComponent {

    public abstract ElectricalContract electricalContract();

    public abstract VisionSubsystem visionSubsystem();

    public abstract SwerveDriveWithJoysticksCommand swerveDriveWithJoysticksCommand();

    public abstract SwerveToPointCommand swerveToPointCommand();

    public abstract SetSwerveMotorControllerPidParametersCommand setSwerveMotorControllerPidParametersCommand();

    public abstract SwerveSteeringMotorPidSubsystem swerveSteeringMotorPidSubsystem();

    public abstract LowerArmSegment lowerArmSegment();

    public abstract UpperArmSegment upperArmSegment();

    public abstract ControlArmsWithJoyStickCommand controlArmsWithJoyStickCommand();

    public abstract ControlEndEffectorPositionCommand controlEndEffectorPositionCommand();

    public abstract UnifiedArmSubsystem unifiedArmSubsystem();

    public abstract ClawSubsystem clawSubsystem();

    public abstract OpenClawCommand openClawCommand();

    public abstract CloseClawCommand closeClawCommand();

    public abstract UnifiedArmMaintainer unifiedArmMaintainer();

    public abstract SimpleSafeArmRouterCommand simpleSafeArmRouterCommand();

    public abstract SimpleXZRouterCommand simpleXZRouterCommand();
}