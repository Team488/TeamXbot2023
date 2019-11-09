
package competition;

import competition.operator_interface.OperatorCommandMap;
import competition.subsystems.SubsystemDefaultCommandMap;
import competition.subsystems.pose.PoseSubsystem;
import xbot.common.command.BaseRobot;

public class Robot extends BaseRobot {

    @Override
    protected void initializeSystems() {
        super.initializeSystems();
        this.injector.getInstance(SubsystemDefaultCommandMap.class);
        this.injector.getInstance(OperatorCommandMap.class);
        registerPeriodicDataSource(this.injector.getInstance(PoseSubsystem.class));
    }

    @Override
    protected void setupInjectionModule() {
        this.injectionModule = new CompetitionModule(true);
    }
}
