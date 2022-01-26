package competition.subsystems.launcher.commands;

import competition.subsystems.launcher.LauncherSubsystem;
import xbot.common.command.BaseMaintainerCommand;
import competition.subsystems.launcher.LauncherSubsystem;

public class launcherMaintainerCommand extends BaseMaintainerCommand{
    
    public LauncherSubsystem launch;

    public launcherMaintainerCommand (LauncherSubsystem launch){
        super(launch, null, null, 0, 0);
        this.launch = launch;
    }
    
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        
    }

    protected void calibratedMachineControlAction(){
        
    }

    protected double getHumanInput(){
        return 0.0;
    }

}
