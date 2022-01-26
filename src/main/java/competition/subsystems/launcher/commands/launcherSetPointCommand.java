package competition.subsystems.launcher.commands;

import xbot.common.command.BaseSetpointCommand;
import xbot.common.command.SupportsSetpointLock;

public class launcherSetPointCommand extends BaseSetpointCommand{

    public launcherSetPointCommand(SupportsSetpointLock system) {
        super(system);
        //TODO Auto-generated constructor stub
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
