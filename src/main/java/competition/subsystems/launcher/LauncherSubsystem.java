package competition.subsystems.launcher;

import xbot.common.command.BaseSetpointSubsystem;
import xbot.common.injection.wpi_factories.CommonLibFactory;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import com.google.inject.Inject;

import competition.electrical_contract.ElectricalContract;

public class LauncherSubsystem extends BaseSetpointSubsystem{

    public DoubleProperty targetval;
    public DoubleProperty currentval;

    //create some motor variables here

    @Inject
    public LauncherSubsystem(CommonLibFactory factory, 
    PropertyFactory pf, ElectricalContract contract){ // use competitioncontract? nah, prob not cause we testing rn

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