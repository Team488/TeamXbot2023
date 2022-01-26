package competition.subsystems.launcher.commands;

import com.google.inject.Inject;

import competition.operator_interface.OperatorInterface;
import competition.subsystems.launcher.LauncherSubsystem;
import xbot.common.command.BaseCommand;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

public class pewPewCommand extends BaseCommand{

    final LauncherSubsystem launch;
    final OperatorInterface oi;
    final DoubleProperty maxPowerProp;

    @Inject
    public pewPewCommand(OperatorInterface oi, 
    LauncherSubsystem launch, PropertyFactory pf){
        this.oi = oi;
        pf.setPrefix(this);

        this.maxPowerProp = pf.createEphemeralProperty("Max Power", 0.0);

        this.launch = launch;
        this.addRequirements(this.launch);
    }

    @Override
    public void initialize() {
        log.info("Initializing");
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        
    }
    
}
