package competition.subsystems.launcher.commands;

import competition.operator_interface.OperatorInterface;
import competition.subsystems.launcher.LauncherSubsystem;
import xbot.common.command.BaseMaintainerCommand;
import xbot.common.injection.wpi_factories.CommonLibFactory;
import xbot.common.properties.PropertyFactory;
import competition.subsystems.launcher.LauncherSubsystem;

public class launcherMaintainerCommand extends BaseMaintainerCommand{
    
    final LauncherSubsystem launch;
    final OperatorInterface oi;

    public launcherMaintainerCommand (LauncherSubsystem launch, OperatorInterface oi, 
    PropertyFactory pf, CommonLibFactory clf){
        super(launch, pf, clf, 50, 0.25); // ask John about the 50 and .25 bc values from 2020 code

        this.oi = oi;
        this.launch = launch;
        this.addRequirements(this.launch);
    }
    
    @Override
    public void initialize() {
        log.info("Initializing");
    }

    protected void calibratedMachineControlAction(){
        
    }

    protected double getHumanInput(){
        return 0.0;
    }

}
