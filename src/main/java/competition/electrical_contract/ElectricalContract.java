package competition.electrical_contract;

import competition.injection.swerve.SwerveInstance;
import xbot.common.injection.electrical_contract.DeviceInfo;
import xbot.common.math.XYPair;

public abstract class ElectricalContract {
    public abstract boolean isDriveReady();

    public abstract boolean areCanCodersReady();

    public abstract DeviceInfo getDriveNeo(SwerveInstance swerveInstance);
    
    public abstract DeviceInfo getSteeringNeo(SwerveInstance swerveInstance);

    public abstract DeviceInfo getSteeringEncoder(SwerveInstance swerveInstance);

    public abstract DeviceInfo getLowerArmLeftMotor();

    public abstract DeviceInfo getLowerArmRightMotor();
    public abstract DeviceInfo getUpperArmLeftMotor();
    public abstract  DeviceInfo getUpperArmRightMotor();
    public abstract  boolean isLowerArmReady();
    public  abstract  boolean isUpperArmReady();
    public abstract boolean isLowerArmEncoderReady();
    public abstract boolean isUpperArmEncoderReady();
    public abstract DeviceInfo getLowerArmEncoder();
    public abstract DeviceInfo getUpperArmEncoder();
    public abstract XYPair getSwerveModuleOffsets(SwerveInstance swerveInstance);
    
    public abstract DeviceInfo getClawSolenoid();
    public abstract DeviceInfo getLowerArmSolenoid();
    public abstract DeviceInfo getCollectorSolenoid();
    public abstract DeviceInfo getCollectorMotor();
    public abstract boolean isCollectorReady();
    
    public abstract DeviceInfo getLowerArmBrakeSolenoid();
}
