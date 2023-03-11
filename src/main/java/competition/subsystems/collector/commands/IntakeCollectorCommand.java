package competition.subsystems.collector.commands;

import competition.operator_interface.OperatorInterface;
import competition.subsystems.collector.CollectorSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class IntakeCollectorCommand extends BaseCommand {
    CollectorSubsystem collector;
    OperatorInterface oi;
    @Inject
    public IntakeCollectorCommand(CollectorSubsystem collector, OperatorInterface oi) {
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
            //if game piece is collected, rumble controller
            if(collector.getGamePieceCollected()){
                oi.operatorGamepad.getRumbleManager().rumbleGamepad(0.5,0.5);
            }
        }
}
