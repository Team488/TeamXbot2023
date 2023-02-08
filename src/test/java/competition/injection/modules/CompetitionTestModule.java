package competition.injection.modules;

import javax.inject.Singleton;

import competition.electrical_contract.ElectricalContract;
import competition.electrical_contract.TestElectricalContract;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.MockPoseSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import dagger.Binds;
import dagger.Module;
import xbot.common.subsystems.drive.BaseDriveSubsystem;
import xbot.common.subsystems.pose.BasePoseSubsystem;

@Module
public abstract class CompetitionTestModule {
    @Binds
    @Singleton
    public abstract ElectricalContract getElectricalContract(TestElectricalContract impl);

    @Binds
    @Singleton
    public abstract PoseSubsystem getPoseSubsystem(MockPoseSubsystem impl);

    @Binds
    @Singleton
    public abstract BasePoseSubsystem getBasePoseSubsystem(PoseSubsystem impl);

    @Binds
    @Singleton
    public abstract BaseDriveSubsystem getDriveSubsystem(DriveSubsystem impl);
}
