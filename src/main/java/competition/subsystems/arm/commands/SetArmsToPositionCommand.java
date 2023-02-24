package competition.subsystems.arm.commands;

import competition.subsystems.arm.UnifiedArmSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import xbot.common.command.BaseSetpointCommand;
import xbot.common.math.XYPair;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;

public class SetArmsToPositionCommand extends BaseSetpointCommand {
    UnifiedArmSubsystem.KeyArmPosition targetPosition;
    UnifiedArmSubsystem.RobotFacing targetFacing;
    private final UnifiedArmSubsystem arms;
    private PoseSubsystem pose;




    @Inject
    public SetArmsToPositionCommand(PropertyFactory propFactory, UnifiedArmSubsystem arms,PoseSubsystem pose){
        this.arms = arms;
        this.pose = pose;
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
        if(pose.getAutomaticScoringInversion().get()){
            targetPosition = automaticScoringInversion(targetPosition);
        }
    }
    public  boolean isFinished(){
        return true;
    }

    public XYPair automaticScoringInversion(UnifiedArmSubsystem.KeyArmPosition keyArm){
        XYPair ArmAngles  = arms.getKeyArmAngles(keyArm, targetFacing)
        Rotation2d rotationpose = pose.getCurrentFieldPose().getHeading();
        boolean isBackwards;


        if(pose.getAlliance() == DriverStation.Alliance.Blue && pose.isAllianceAwareField()){
            if(rotationpose.getDegrees() > -90 && rotationpose.getDegrees() < 90){
                isBackwards = true;
            }
            else{
                isBackwards = false;
            }
        }
        else{
            if(rotationpose.getDegrees() > -90 && rotationpose.getDegrees() < 90){
                isBackwards = false;
            }
            else{
                isBackwards = true;
            }
        }
        if(isBackwards){
            ArmAngles = new XYPair(-ArmAngles.x,- ArmAngles.y);
        }
        return ArmAngles;
    }
}