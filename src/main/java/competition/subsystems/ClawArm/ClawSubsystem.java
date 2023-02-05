package competition.subsystems.ClawArm;

import competition.electrical_contract.ElectricalContract;
import xbot.common.command.BaseSubsystem;
import xbot.common.controls.actuators.XSolenoid;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public  class ClawSubsystem extends BaseSubsystem {
    public XSolenoid ClawSolenoid;

    public enum ClawState{

        CloseCLaw,
        OpenClaw

    }

//

    // true = open || false = close
    private void ChangeClaw(ClawState state){
        if(state == ClawState.OpenClaw){
            //open claw
            ClawSolenoid.setOn(true);

        }
        else if(state == ClawState.CloseCLaw){
            //close claw
            ClawSolenoid.setOn(false);
        }


    }
    @Inject
    public ClawSubsystem(XSolenoid.XSolenoidFactory solenoidFactory, ElectricalContract eContract, PropertyFactory propFactory){
        this.ClawSolenoid = solenoidFactory.create(eContract.getClawSolenoid().channel);
        ClawSolenoid.setInverted(eContract.getClawSolenoid().inverted);


        

    }

    public boolean GetClawState(){
        return ClawSolenoid.getAdjusted();

    }

    public  void Open(){
        ChangeClaw(ClawState.OpenClaw);
    }

    public void Close() {ChangeClaw(ClawState.CloseCLaw);}

}