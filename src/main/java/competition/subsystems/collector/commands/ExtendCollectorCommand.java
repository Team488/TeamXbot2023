package competition.subsystems.collector.commands;

import competition.subsystems.collector.CollectorSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class ExtendCollectorCommand extends BaseCommand {
    CollectorSubsystem collector;
    @Inject
    public ExtendCollectorCommand(CollectorSubsystem collector){
        this.collector = collector;
    }
    @Override
    public void initialize() {
        log.info("Initializing");
        collector.extend();
    }

    @Override
    public void execute() {

    }

    public boolean isFinished(){
        return true;
    }

}
