package competition.subsystems.claw;

import competition.operator_interface.OperatorInterface;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class OpenClawCommand extends BaseCommand{

    OperatorInterface oi;
    ClawSubsystem clawSubsystem;

    @Inject
    public OpenClawCommand(OperatorInterface oi, ClawSubsystem clawSubsystem){
        this.oi = oi;
        this.clawSubsystem = clawSubsystem;
    }

    @Override
    public void initialize() {
        log.info("Initializing");
    }

    @Override
    public void execute() {
        clawSubsystem.open();
    }
}