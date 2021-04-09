package competition.injection;

import competition.electrical_contract.CompetitionContract;
import competition.electrical_contract.ElectricalContract;
import xbot.common.injection.SimulatorModule;
import xbot.common.subsystems.drive.BaseDriveSubsystem;
import xbot.common.subsystems.pose.BasePoseSubsystem;


public class SimulationModule extends SimulatorModule {

    boolean isPractice;

    public SimulationModule() {
        this.isPractice = true;
    }
    
    @Override
    protected void configure() {
        super.configure();
        this.bind(ElectricalContract.class).to(CompetitionContract.class);
        this.bind(BasePoseSubsystem.class).to(CompetitionModule.CHOSEN_POSE_SUBSYSTEM);
        this.bind(BaseDriveSubsystem.class).to(CompetitionModule.CHOSEN_DRIVE_SUBSYSTEM);
    }
}
