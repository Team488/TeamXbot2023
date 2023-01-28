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
    public abstract XYPair getSwerveModuleOffsets(SwerveInstance swerveInstance);
    public abstract DeviceInfo getClawMotor();
}
