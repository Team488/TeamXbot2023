package competition.subsystems.drive;

import competition.BaseCompetitionTest;
import competition.subsystems.drive.commands.VelocityMaintainerCommand;
import competition.subsystems.pose.PoseSubsystem;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class VelocityMaintainerCommandTest extends BaseCompetitionTest {

    PoseSubsystem pose;
    DriveSubsystem drive;
    VelocityMaintainerCommand maintainer;

    @Override
    public void setUp() {
        super.setUp();

        pose = (PoseSubsystem)getInjectorComponent().poseSubsystem();
        drive = (DriveSubsystem)getInjectorComponent().driveSubsystem();
        maintainer = getInjectorComponent().velocityMaintainerCommand();
    }

    @Test
    public void testRobotOrientedVelocity() {
        // Test forward
        pose.setCurrentHeading(-90);
        pose.velocityX.set(0);
        pose.velocityY.set(-1);
        assertEquals(1, pose.getRobotOrientedXVelocity(), 0.001);

        pose.setCurrentHeading(-135);
        pose.velocityY.set(-Math.sqrt(2));
        pose.velocityX.set(-Math.sqrt(2));
        assertEquals(2, pose.getRobotOrientedXVelocity(), 0.001);

        pose.setCurrentHeading(135);
        pose.velocityX.set(-Math.sqrt(2));
        pose.velocityY.set(Math.sqrt(2));
        assertEquals(2, pose.getRobotOrientedXVelocity(), 0.001);

        // Test backward
        pose.setCurrentHeading(-90);
        pose.velocityX.set(0);
        pose.velocityY.set(1);
        assertEquals(-1, pose.getRobotOrientedXVelocity(), 0.001);

        pose.setCurrentHeading(-135);
        pose.velocityY.set(Math.sqrt(2));
        pose.velocityX.set(Math.sqrt(2));
        assertEquals(-2, pose.getRobotOrientedXVelocity(), 0.001);

        pose.setCurrentHeading(135);
        pose.velocityX.set(Math.sqrt(2));
        pose.velocityY.set(-Math.sqrt(2));
        assertEquals(-2, pose.getRobotOrientedXVelocity(), 0.001);
    }
}
