package competition.subsystems.arm.commands;

import competition.subsystems.arm.LowerArmSubsystem;
import competition.subsystems.arm.UpperArmSubsystem;
import xbot.common.command.BaseSetpointCommand;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;
import xbot.common.properties.DoubleProperty;

public class SetArmsToPositionCommand extends BaseSetpointCommand {
    DoubleProperty targetPositionProp = null;
    final LowerArmSubsystem lowerArm;
    final UpperArmSubsystem upperArm;
    public enum TargetPosition{
        lowerGoal,
        midGoal,
        highGoal,
        fullyRetracted
    }
    @Inject
    public SetArmsToPositionCommand(PropertyFactory propFactory, LowerArmSubsystem lowerArm, UpperArmSubsystem upperArm){
        this.lowerArm = lowerArm;
        this.upperArm = upperArm;
    }

    public SetArmsToPositionCommand setTargetPosition(TargetPosition targetPosition){
        switch (targetPosition){
            case lowerGoal:
                targetPositionProp = upperArm.positionLowerGoalProperty;
                targetPositionProp = lowerArm.positionLowerGoalProperty;
            case midGoal:
                targetPositionProp = upperArm.positionMidGoalProperty;
                targetPositionProp = lowerArm.positionMidGoalProperty;
                break;
            case highGoal:
                targetPositionProp = upperArm.positionHighGoalProperty;
                targetPositionProp = lowerArm.positionHighGoalProperty;
                break;
            case fullyRetracted:
                targetPositionProp = upperArm.positionFullyRetractedProperty;
                targetPositionProp = lowerArm.positionFullyRetractedProperty;
                break;
            default:
                throw  new RuntimeException("Unknown Target Position" + targetPosition);
        }
        this.setName("SetArmsToTargetPositionCommand" + targetPosition);
        return this;
        }
    @Override
    public void initialize() {
        log.info("Initializing with target position");
        if(targetPositionProp != null){
            this.upperArm.setTargetValue(targetPositionProp.get());
            this.lowerArm.setTargetValue(targetPositionProp.get());
        }
    }
    @Override
    public void execute(){

    }
    public  boolean isFinished(){
        return true;
    }
}
