package competition.injection.components;

import javax.inject.Singleton;

import competition.electrical_contract.ElectricalContract;
import competition.injection.modules.CommonModule;
import competition.injection.modules.CompetitionTestModule;
import competition.subsystems.ClawArm.ClawSubsystem;
import competition.subsystems.ClawArm.CloseClawCommand;
import competition.subsystems.ClawArm.OpenClawCommand;
import competition.subsystems.arm.LowerArmSubsystem;
import competition.subsystems.arm.UpperArmSubsystem;
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

@Singleton
@Component(modules = { UnitTestModule.class, MockDevicesModule.class, MockControlsModule.class, CompetitionTestModule.class, CommonModule.class })
public abstract class CompetitionTestComponent extends BaseRobotComponent {

    public abstract ElectricalContract electricalContract();

    public abstract VisionSubsystem visionSubsystem();

    public abstract SwerveDriveWithJoysticksCommand swerveDriveWithJoysticksCommand();

    public abstract SwerveToPointCommand swerveToPointCommand();

    public abstract SetSwerveMotorControllerPidParametersCommand setSwerveMotorControllerPidParametersCommand();

    public abstract SwerveSteeringMotorPidSubsystem swerveSteeringMotorPidSubsystem();

    public abstract LowerArmSubsystem lowerArmSubsystem();

    public abstract UpperArmSubsystem upperArmSubsystem();

    public  abstract ControlArmsWithJoyStickCommand controlArmsWithJoyStickCommand();

    public abstract ClawSubsystem clawSubsystem();

    public abstract OpenClawCommand openClawCommand();

    public abstract CloseClawCommand closeClawCommand();


}