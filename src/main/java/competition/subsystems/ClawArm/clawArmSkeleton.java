package competition.subsystems.ClawArm;

import competition.electrical_contract.ElectricalContract;
import xbot.common.controls.actuators.XCANSparkMax;
import xbot.common.controls.actuators.XCANTalon;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;

public  class clawArmSkeleton{
    private XCANTalon ClawMotor;
    private prefix
    ;
    private static String CLOSECONE = "close_cone";
    private static String CLOSECUBE = "close_cube";
    private static String OPENARM = "open_arm";

    // true = open || false = close
    private static void ChangeClaw(String Answer){
        if(Answer.equals(OPENARM)){
            //open claw

        }
        else if(Answer.equals(CLOSECONE)){
            //close claw to cone length
        }
        else if(Answer.equals(CLOSECUBE)){
            //close claw to cube length
        }

    }
    @Inject
    public clawArmSkeleton(XCANSparkMax.XCANSparkMaxFactory sparkMaxFactor, ElectricalContract eContract, PropertyFactory propFactory){
        this.ClawMotor = sparkMaxFactor.create(eContract.getClawMotor(),this.getPrefix(),"lowerArmLeftMotor");
        propFactory.setPrefix(this.getPrefix());

        this.powerProp = propFactory.createPersistentProperty("Standard Motor Power", 1);
        extendLimit = propFactory.createPersistentProperty("extendLimit",0);
        retractLimit = propFactory.createPersistentProperty("retractLimit",0);
    }

    public static void Open(){
        ChangeClaw(OPENARM);
    }

    public static void GrabCone(){
        ChangeClaw(CLOSECONE);
    }

    public static void GrabCube(){
        ChangeClaw(CLOSECUBE);
    }
}