package competition.electrical_contract;

import competition.injection.swerve.SwerveInstance;
import competition.subsystems.pose.PoseSubsystem;
import xbot.common.injection.electrical_contract.DeviceInfo;
import xbot.common.math.XYPair;

import javax.inject.Inject;

public class PracticeContract2022 extends CompetitionContract {

    protected final double simulationScalingValue = 256.0 * PoseSubsystem.INCHES_IN_A_METER;

    @Inject
    public PracticeContract2022() {}

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
        switch (swerveInstance.getLabel()) {
            case "FrontLeftDrive":
                return new DeviceInfo(31, false, simulationScalingValue);

            case "FrontRightDrive":
                return new DeviceInfo(28, false, simulationScalingValue);

            case "RearLeftDrive":
                return new DeviceInfo(38, false, simulationScalingValue);

            case "RearRightDrive":
                return new DeviceInfo(21, false, simulationScalingValue);

            default:
                return null;
        }
    }

    @Override
    public DeviceInfo getSteeringNeo(SwerveInstance swerveInstance) {
        double simulationScalingValue = 1.0;

        switch (swerveInstance.getLabel()) {
            case "FrontLeftDrive":
                return new DeviceInfo(30, false, simulationScalingValue);

            case "FrontRightDrive":
                return new DeviceInfo(29, false, simulationScalingValue);

            case "RearLeftDrive":
                return new DeviceInfo(39, false, simulationScalingValue);

            case "RearRightDrive":
                return new DeviceInfo(20, false, simulationScalingValue);

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
    public boolean isUpperArmReady() {
        return false;
    }

    @Override
    public boolean isLowerArmReady() {
        return false;
    }

    @Override
    public boolean isUpperArmEncoderReady() {
        return false;
    }

    @Override
    public boolean isLowerArmEncoderReady() {
        return false;
    }

    @Override
    public boolean isCollectorReady() {
        return false;
    }
}
