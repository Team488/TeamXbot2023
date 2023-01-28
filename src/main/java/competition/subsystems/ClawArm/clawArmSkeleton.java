package competition.subsystems.ClawArm;

import competition.electrical_contract.ElectricalContract;
import xbot.common.command.BaseSubsystem;
import xbot.common.controls.actuators.XCANSparkMax;
import xbot.common.controls.actuators.XCANTalon;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;

public  class clawArmSkeleton extends BaseSubsystem {
    private static XCANSparkMax ClawMotor;

    public enum ClawState{
        closeCube,
        closeCone,
        openArm

    }

//

    private  final DoubleProperty Open;
    private  final DoubleProperty closedCone;

    private  final DoubleProperty closedCube;

    // true = open || false = close
    private void ChangeClaw(ClawState state){
        if(state == ClawState.openArm){
            //open claw
            ClawMotor.set(Open.get());

        }
        else if(state == ClawState.closeCone){
            //close claw to cone length
            ClawMotor.set(closedCone.get());
        }
        else if(state == ClawState.closeCube){
            //close claw to cube length
            ClawMotor.set(closedCube.get());
        }

    }
    @Inject
    public clawArmSkeleton(XCANSparkMax.XCANSparkMaxFactory sparkMaxFactor, ElectricalContract eContract, PropertyFactory propFactory){
        this.ClawMotor = sparkMaxFactor.create(eContract.getClawMotor(),this.getPrefix(),"lowerArmLeftMotor");
        propFactory.setPrefix(this.getPrefix());

        
        Open = propFactory.createPersistentProperty("Open",0);
        closedCone = propFactory.createPersistentProperty("closedCone",0);
        closedCube = propFactory.createPersistentProperty("closedCube", 0);
    }

    public  void Open(){
        ChangeClaw(ClawState.openArm);
    }

    public  void GrabCone(){
        ChangeClaw(ClawState.closeCone);
    }

    public  void GrabCube(){
        ChangeClaw(ClawState.closeCube);
    }
}