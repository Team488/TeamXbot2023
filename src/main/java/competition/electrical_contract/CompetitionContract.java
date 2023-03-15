package competition.electrical_contract;

import javax.inject.Inject;

import competition.injection.swerve.SwerveInstance;
import competition.subsystems.pose.PoseSubsystem;
import xbot.common.injection.electrical_contract.DeviceInfo;
import xbot.common.math.XYPair;

public class CompetitionContract extends ElectricalContract {

    protected final double simulationScalingValue = 256.0 * PoseSubsystem.INCHES_IN_A_METER;

    @Inject
    public CompetitionContract() {}

    @Override
    public boolean isDriveReady() {
        return true;
    }

    @Override
    public boolean areCanCodersReady() {
        return true;
    }

    @Override
    public DeviceInfo getDriveNeo(SwerveInstance swerveInstance) {
        double simulationScalingValue = 2;
        switch (swerveInstance.getLabel()) {
            case "FrontLeftDrive":
                return new DeviceInfo(getDriveControllerName(swerveInstance),31, false, simulationScalingValue);

            case "FrontRightDrive":
                return new DeviceInfo(getDriveControllerName(swerveInstance),29, false, simulationScalingValue);

            case "RearLeftDrive":
                return new DeviceInfo(getDriveControllerName(swerveInstance),38, false, simulationScalingValue);

            case "RearRightDrive":
                return new DeviceInfo(getDriveControllerName(swerveInstance),21, false, simulationScalingValue);

            default:
                return null;
        }
    }

    @Override
    public DeviceInfo getSteeringNeo(SwerveInstance swerveInstance) {
        double simulationScalingValue = 180 / Math.PI / 28.1502912;

        switch (swerveInstance.getLabel()) {
            case "FrontLeftDrive":
                return new DeviceInfo(getSteeringControllerName(swerveInstance),30, false, simulationScalingValue);

            case "FrontRightDrive":
                return new DeviceInfo(getSteeringControllerName(swerveInstance),28, false, simulationScalingValue);

            case "RearLeftDrive":
                return new DeviceInfo(getSteeringControllerName(swerveInstance),39, false, simulationScalingValue);

            case "RearRightDrive":
                return new DeviceInfo(getSteeringControllerName(swerveInstance),20, false, simulationScalingValue);

            default:
                return null;
        }
    }

    @Override
    public DeviceInfo getSteeringEncoder(SwerveInstance swerveInstance) {
        double simulationScalingValue = 180.0 / Math.PI;

        switch (swerveInstance.getLabel()) {
            case "FrontLeftDrive":
                return new DeviceInfo(getSteeringEncoderName(swerveInstance),51, false, simulationScalingValue);

            case "FrontRightDrive":
                return new DeviceInfo(getSteeringEncoderName(swerveInstance),52, false, simulationScalingValue);

            case "RearLeftDrive":
                return new DeviceInfo(getSteeringEncoderName(swerveInstance),53, false, simulationScalingValue);

            case "RearRightDrive":
                return new DeviceInfo(getSteeringEncoderName(swerveInstance),54, false, simulationScalingValue);

            default:
                return null;
        }
    }

    @Override
    public XYPair getSwerveModuleOffsets(SwerveInstance swerveInstance) {
        switch (swerveInstance.getLabel()) {
            case "FrontLeftDrive":
                return new XYPair(15, 15);
            case "FrontRightDrive":
                return new XYPair(15, -15);
            case "RearLeftDrive":
                return new XYPair(-15, 15);
            case "RearRightDrive":
                return new XYPair(-15, -15);
            default:
                return new XYPair(0, 0);
        }
    }

    @Override
    public boolean isForwardAprilCamReady() {
        return true;
    }

    @Override
    public DeviceInfo getLowerArmLeftMotor() {
        return new DeviceInfo("LowerArmLeft",37,true);
    }

    @Override
    public DeviceInfo getLowerArmRightMotor() {
        return new DeviceInfo("LowerArmRight",22,false);
    }

    public DeviceInfo getUpperArmLeftMotor(){
        return new DeviceInfo("UpperArmLeft",35,true);
    }

    @Override
    public DeviceInfo getUpperArmRightMotor() {
        return new DeviceInfo("UpperArmRight",24,true);
    }

    public boolean isLowerArmReady() { return true;}

    public boolean isUpperArmReady() { return true;}

    @Override
    public boolean isLowerArmEncoderReady() {
        return true;
    }

    @Override
    public boolean isUpperArmEncoderReady() {
        return true;
    }

    @Override
    public DeviceInfo getLowerArmEncoder() {
        return new DeviceInfo("UnifiedArmSubsystem/LowerArm",0, true);
    }

    @Override
    public DeviceInfo getUpperArmEncoder() {
        return new DeviceInfo("UnifiedArmSubsystem/UpperArm",1, true);
    }

    public DeviceInfo getClawSolenoid() {return new DeviceInfo("ClawSubsystem",0, false);}

    @Override
    public DeviceInfo getLeftClawMotor() {
        return new DeviceInfo("LeftClaw", 34, true, 1);
    }

    @Override
    public DeviceInfo getRightClawMotor() {
        return new DeviceInfo("RightClaw", 33, false, 1);
    }

    @Override
    public boolean areClawMotorsReady() {
        return true;
    }

    @Override
    public DeviceInfo getLowerArmBrakeSolenoid() {
        return new DeviceInfo("LowerArmBrake", 1, false);
    }

    public DeviceInfo getCollectorMotor(){ return new DeviceInfo("Collector",25,false);}

    @Override
    public boolean isCollectorReady() { return true; }

    public DeviceInfo getCollectorSolenoid(){ return new DeviceInfo("Collector", 2,false);}

    @Override
    public DeviceInfo getArduinoDio0() {
        return new DeviceInfo("Arduino0",8);
    }

    @Override
    public DeviceInfo getArduinoDio1() {
        return new DeviceInfo("Arduino1", 9);
    }

    @Override
    public DeviceInfo getArduinoDio2() {
        return new DeviceInfo("Arduino2", 10);
    }

    @Override
    public DeviceInfo getArduinoDio3() {
        return new DeviceInfo("Arduino3", 11); // something on the navX, just out of the way
    }

    @Override
    public DeviceInfo getArduinoAllianceDio() {
        return new DeviceInfo("ArduinoAlliance", 4);
    }

    public DeviceInfo getPressureSensor() {return new DeviceInfo("PressurSensor", 3);}
}
