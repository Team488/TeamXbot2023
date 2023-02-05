package competition.subsystems.arm.commands;

import competition.subsystems.arm.UnifiedArmSubsystem;
import xbot.common.command.BaseSetpointCommand;
import xbot.common.math.XYPair;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;
import xbot.common.properties.DoubleProperty;

public class SetArmsToPositionCommand extends BaseSetpointCommand {
    UnifiedArmSubsystem.KeyArmPosition targetPosition;
    private final UnifiedArmSubsystem arms;

    @Inject
    public SetArmsToPositionCommand(PropertyFactory propFactory, UnifiedArmSubsystem arms){
        this.arms = arms;
    }

    public void setTargetPosition(UnifiedArmSubsystem.KeyArmPosition keyArmPosition){
        this.targetPosition = keyArmPosition;
        this.setName("SetArmsTo" + keyArmPosition +"PositionCommand");
    }
    @Override
    public void initialize() {
        log.info("Initializing with target position");
        if(targetPosition != null){
            arms.setTargetValue(arms.getKeyArmPosition(targetPosition));
        }
    }
    @Override
    public void execute(){

    }
    public  boolean isFinished(){
        return true;
    }
}