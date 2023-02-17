package competition.subsystems.collector.commands;

import competition.subsystems.collector.CollectorSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class ForwardCollectorCommand extends BaseCommand {
    CollectorSubsystem collector;
    @Inject
    public ForwardCollectorCommand(CollectorSubsystem collector) {
        this.collector = collector;
        }
        @Override
        public void initialize() {
            log.info("Initializing");
        }

        @Override
        public void execute() {
            collector.intake();
        }
}
