package competition.injection.components;

import javax.inject.Singleton;

import competition.injection.modules.CommonModule;
import competition.injection.modules.CompetitionModule;
import competition.injection.swerve.SwerveComponentHolder;
import competition.subsystems.arm.UnifiedArmSubsystem;
import dagger.Component;
import xbot.common.injection.modules.RealControlsModule;
import xbot.common.injection.modules.RealDevicesModule;
import xbot.common.injection.modules.RobotModule;

@Singleton
@Component(modules = { RobotModule.class, RealDevicesModule.class, RealControlsModule.class, CompetitionModule.class, CommonModule.class })
public abstract class RobotComponent extends BaseRobotComponent {
    public abstract SwerveComponentHolder swerveComponents();

    public abstract UnifiedArmSubsystem unifiedArmSubsystem();
}
