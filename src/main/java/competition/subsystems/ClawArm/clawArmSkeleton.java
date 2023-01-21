package competition.subsystems.ClawArm;

import xbot.common.controls.actuators.XCANTalon;

public  class clawArmSkeleton{
    private XCANTalon ClawMotor = new XCANTalon(11);
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