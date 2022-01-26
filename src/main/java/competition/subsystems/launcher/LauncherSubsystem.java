package competition.subsystems.launcher;

import xbot.common.command.BaseSetpointSubsystem;
import xbot.common.controls.actuators.XCANSparkMax;
import xbot.common.injection.wpi_factories.CommonLibFactory;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import com.google.inject.Inject;

import competition.electrical_contract.ElectricalContract;

public class LauncherSubsystem extends BaseSetpointSubsystem{

    public DoubleProperty targetval;
    public DoubleProperty currentval;

    //Questions need to answer:
    // just a leader and follow or more/less motors?
    public XCANSparkMax leader;
    private XCANSparkMax follower;
    ElectricalContract contract;

    @Inject
    public LauncherSubsystem(CommonLibFactory factory, 
    PropertyFactory pf, ElectricalContract contract){ // use competitioncontract? nah, prob not cause we testing rn
        log.info("creating LauncherSubsystem");
        this.contract = contract;

        // if(contract.isLauncherReady()){
        //     // this.leader = factory.createCANSparkMax();
        //     // this.follower = factory.createCANSparkMax();

        //     // this.leader.enableVoltageCompensation(12);

        //     leader.burnFlash();
        //     follower.burnFlash();
        // }

    }
    
    public double getCurrentValue(){
        return currentval.get();
    }
    
    public double getTargetValue(){
        return targetval.get();
    }

    public void setPower(double power){
        
    }

    public void setTargetValue(double value){

    }
    
    public boolean isCalibrated(){
        // tests? should have a statement where
        // we use IsMaintainerAtGoal() to check.
        return false; // TODO: TEMPORARY
    }
}