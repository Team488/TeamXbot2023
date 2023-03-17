package competition.subsystems.drive.swerve.commands;

import competition.auto_programs.AutoLandmarks;
import competition.subsystems.drive.commands.SwerveToNearestScoringPositionCommand;
import competition.subsystems.drive.commands.SwerveToPointCommand;
import competition.subsystems.pose.MockPoseSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import junitparams.JUnitParamsRunner;
import junitparams.Parameters;
import org.junit.Test;
import org.junit.runner.RunWith;
import xbot.common.math.WrappedRotation2d;
import xbot.common.math.XYPair;

import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

@RunWith(JUnitParamsRunner.class)
public class SwerveToNearestScoringPositionCommandTest extends BaseFullSwerveTest {
    
    SwerveToNearestScoringPositionCommand command;
    MockPoseSubsystem pose;

    @Override
    public void setUp() {
        super.setUp();
        command = getInjectorComponent().swerveToNearestScoringPositionCommand();
        pose = (MockPoseSubsystem) getInjectorComponent().poseSubsystem();
        drive.getPositionalPid().setP(1);
        drive.getPositionalPid().setI(0);
        drive.getPositionalPid().setD(0);
    }

    @Test
    @Parameters({
            "0, 0, 0",
            "0, 20, 0",
            "0, 80, 3",
            "0, 200, 8",
            "650, 0, 0",
            "650, 20, 0",
            "650, 80, 3",
            "650, 200, 8"
    })
    public void findNearestPositionForBlueAllianceTest(double currentX, double currentY, int expectedNearestPositionIndex) {
        List<Pose2d> scoringPoses = SwerveToNearestScoringPositionCommand.getScoringPositionPoses(DriverStation.Alliance.Blue);

        Pose2d nearestPose = SwerveToNearestScoringPositionCommand.findNearestScoringPosition(
                new Pose2d(new Translation2d(currentX, currentY), new Rotation2d(0)), DriverStation.Alliance.Blue);

        assertEquals(scoringPoses.get(expectedNearestPositionIndex), nearestPose);
        assertEquals(Rotation2d.fromDegrees(-180), nearestPose.getRotation());
        assertTrue(nearestPose.getX() < 325);
    }

    @Test
    @Parameters({
            "0, 0, 0",
            "0, 20, 0",
            "0, 80, 3",
            "0, 200, 8",
            "650, 0, 0",
            "650, 20, 0",
            "650, 80, 3",
            "650, 200, 8"
    })
    public void findNearestPositionForRedAllianceTest(double currentX, double currentY, int expectedNearestPositionIndex) {
        List<Pose2d> scoringPoses = SwerveToNearestScoringPositionCommand.getScoringPositionPoses(DriverStation.Alliance.Red);

        Pose2d nearestPose = SwerveToNearestScoringPositionCommand.findNearestScoringPosition(
                new Pose2d(new Translation2d(currentX, currentY), new Rotation2d(0)), DriverStation.Alliance.Red);

        assertEquals(scoringPoses.get(expectedNearestPositionIndex), nearestPose);
        assertEquals(Rotation2d.fromDegrees(0), nearestPose.getRotation());
        assertTrue(nearestPose.getX() > 325);
    }

    @Test
    @Parameters({
            "0.0",
            "1.0",
            "-1.0"
    })
    public void testSetTarget(double initialXOffset) {
        pose.setAlliance(DriverStation.Alliance.Blue);
        pose.setCurrentPosition(
                AutoLandmarks.blueScoringPositionOne.getX() + initialXOffset,
                AutoLandmarks.blueScoringPositionOne.getY());
        pose.setCurrentHeading(AutoLandmarks.blueScoringPositionOne.getRotation().getDegrees());
        setAllSteeringModuleAngles(0);

        command.initialize();
        command.execute();

        assertEquals(2, command.getKeyPoints().size());
        assertEquals(AutoLandmarks.blueScoringPositionOne.getX(), command.getKeyPoints().get(1).keyPose.getX(), 0.01);
        assertEquals(AutoLandmarks.blueScoringPositionOne.getY(), command.getKeyPoints().get(1).keyPose.getY(), 0.01);
        assertEquals(AutoLandmarks.blueScoringPositionOne.getRotation().getDegrees(), command.getKeyPoints().get(1).keyPose.getRotation().getDegrees(), 0.01);

        assertTrue(command.getKeyPoints().get(0).keyPose.getX() > command.getKeyPoints().get(1).keyPose.getX());
    }

    @Test
    public void testSetTargetTooFarAway() {
        pose.setAlliance(DriverStation.Alliance.Blue);
        pose.setCurrentPosition(
                AutoLandmarks.blueScoringPositionOne.getX() + 300,
                AutoLandmarks.blueScoringPositionOne.getY());
        pose.setCurrentHeading(27);
        setAllSteeringModuleAngles(0);

        command.initialize();
        command.execute();

        assertEquals(1, command.getKeyPoints().size());
        assertEquals(pose.getCurrentPose2d().getX(), command.getKeyPoints().get(0).keyPose.getX(), 0.01);
        assertEquals(pose.getCurrentPose2d().getY(), command.getKeyPoints().get(0).keyPose.getY(), 0.01);
        assertEquals(pose.getCurrentPose2d().getRotation().getDegrees(), command.getKeyPoints().get(0).keyPose.getRotation().getDegrees(), 0.01);
    }

}
