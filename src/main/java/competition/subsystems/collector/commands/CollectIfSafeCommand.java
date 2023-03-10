package competition.subsystems.collector.commands;

import competition.subsystems.arm.UnifiedArmSubsystem;
import competition.subsystems.collector.CollectorSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class CollectIfSafeCommand extends BaseCommand {

    CollectorSubsystem collector;
    UnifiedArmSubsystem arms;

    @Inject
    public CollectIfSafeCommand(CollectorSubsystem collector, UnifiedArmSubsystem arms) {
        this.collector = collector;
        this.arms = arms;
        this.addRequirements(collector);
    }


    @Override
    public void initialize() {
        log.info("Initializing");
    }

    @Override
    public void execute() {
        collector.intake();

        if (arms.getCurrentXZCoordinates().y > 16) {
            collector.extend();
        }
    }

    @Override
    public void end(boolean interrupted) {
        collector.retract();
    }
}
