package competition.injection.components;

import javax.inject.Singleton;

import competition.electrical_contract.ElectricalContract;
import competition.injection.modules.CommonModule;
import competition.injection.modules.CompetitionTestModule;
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
}