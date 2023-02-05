package competition.subsystems.simple;

import competition.operator_interface.OperatorInterface;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class SimpleSetPowerCommand extends BaseCommand {

    private final SimpleSubsystem simpleSubsystem;
    private final OperatorInterface oi;

    @Inject
    SimpleSetPowerCommand(SimpleSubsystem simpleSubsystem, OperatorInterface oi) {
        this.simpleSubsystem = simpleSubsystem;
        this.oi = oi;
        this.requires(simpleSubsystem);
    }

    @Override
    public void initialize() {
        log.info("Initializing");
    }

    @Override
    public void execute() {
        simpleSubsystem.setPower(oi.operatorGamepad.getLeftVector().y);
    }
}
