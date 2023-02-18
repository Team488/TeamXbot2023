package competition.subsystems.arm.commands;

import competition.subsystems.arm.UnifiedArmSubsystem;
import xbot.common.command.BaseSetpointCommand;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;

public class SetArmsToPositionCommand extends BaseSetpointCommand {
    UnifiedArmSubsystem.KeyArmPosition targetPosition;
    UnifiedArmSubsystem.RobotFacing targetFacing;
    private final UnifiedArmSubsystem arms;

    @Inject
    public SetArmsToPositionCommand(PropertyFactory propFactory, UnifiedArmSubsystem arms){
        this.arms = arms;
    }

    public void setTargetPosition(UnifiedArmSubsystem.KeyArmPosition keyArmPosition, UnifiedArmSubsystem.RobotFacing facing){
        this.targetPosition = keyArmPosition;
        this.targetFacing = facing;
        this.setName("SetArmsTo" + keyArmPosition +"PositionCommand");
    }
    @Override
    public void initialize() {
        log.info("Initializing with target position");
        if(targetPosition != null){
            arms.setTargetValue(arms.getKeyArmCoordinates(targetPosition, targetFacing));
        }
    }
    @Override
    public void execute(){

    }
    public  boolean isFinished(){
        return true;
    }
}