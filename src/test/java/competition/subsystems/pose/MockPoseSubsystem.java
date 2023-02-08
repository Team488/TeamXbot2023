package competition.subsystems.pose;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.vision.VisionSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import xbot.common.controls.sensors.XGyro;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;

public class MockPoseSubsystem extends PoseSubsystem {

    private DriverStation.Alliance alliance = DriverStation.Alliance.Blue;

    @Inject
    public MockPoseSubsystem(XGyro.XGyroFactory gyroFactory, PropertyFactory propManager, DriveSubsystem drive, VisionSubsystem vision) {
        super(gyroFactory, propManager, drive, vision);
    }

    @Override
    public DriverStation.Alliance getAlliance() {
        return alliance;
    }

    public void setAlliance(DriverStation.Alliance alliance) {
        this.alliance = alliance;
    }
}
