package competition;

import competition.electrical_contract.CompetitionContract;
import competition.electrical_contract.ElectricalContract;
import competition.injection.CompetitionModule;
import xbot.common.injection.UnitTestModule;
import xbot.common.subsystems.drive.BaseDriveSubsystem;
import xbot.common.subsystems.pose.BasePoseSubsystem;

public class CompetitionTestModule extends UnitTestModule {
    
    @Override
    protected void configure() {
        super.configure();
        this.bind(BasePoseSubsystem.class).to(CompetitionModule.CHOSEN_POSE_SUBSYSTEM);
        this.bind(BaseDriveSubsystem.class).to(CompetitionModule.CHOSEN_DRIVE_SUBSYSTEM);
        this.bind(ElectricalContract.class).to(CompetitionContract.class);
    }
}