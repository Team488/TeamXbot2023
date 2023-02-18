package competition.subsystems.claw;

import competition.operator_interface.OperatorInterface;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class CloseClawCommand extends BaseCommand{

    OperatorInterface oi;
    ClawSubsystem clawSubsystem;
    @Inject
    public CloseClawCommand(OperatorInterface oi, ClawSubsystem clawSubsystem){
        this.oi = oi;
        this.clawSubsystem = clawSubsystem;
    }

    @Override
    public void initialize() {
        log.info("Initializing");
    }

    @Override
    public void execute() {
        clawSubsystem.close();
    }
}