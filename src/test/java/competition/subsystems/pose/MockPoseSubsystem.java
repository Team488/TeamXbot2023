package competition.subsystems.pose;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.vision.VisionSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import xbot.common.controls.sensors.XGyro;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;

public class MockPoseSubsystem extends PoseSubsystem {

    @Inject
    public MockPoseSubsystem(XGyro.XGyroFactory gyroFactory, PropertyFactory propManager, DriveSubsystem drive, VisionSubsystem vision) {
        super(gyroFactory, propManager, drive, vision);
    }

    public void setAlliance(DriverStation.Alliance alliance) {
        cachedAlliance = alliance;
    }
}
