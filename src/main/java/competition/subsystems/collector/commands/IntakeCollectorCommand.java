package competition.subsystems.collector.commands;

import competition.operator_interface.OperatorInterface;
import competition.subsystems.collector.CollectorSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class IntakeCollectorCommand extends BaseCommand {
    CollectorSubsystem collector;
    OperatorInterface oi;
    @Inject
    public IntakeCollectorCommand(CollectorSubsystem collector) {
        this.collector = collector;
        this.oi = oi;
        addRequirements(collector);
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
