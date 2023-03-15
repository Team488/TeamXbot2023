package competition.subsystems.drive.swerve.commands;

import competition.subsystems.drive.commands.SwerveToNearestScoringPositionCommand;
import competition.subsystems.drive.commands.SwerveToNextScoringPositionCommand;
import competition.subsystems.pose.MockPoseSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import junitparams.JUnitParamsRunner;
import junitparams.Parameters;
import org.junit.Test;
import org.junit.runner.RunWith;

import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

@RunWith(JUnitParamsRunner.class)
public class SwerveToNextScoringPositionCommandTest extends BaseFullSwerveTest {

    SwerveToNextScoringPositionCommand command;
    MockPoseSubsystem pose;

    @Override
    public void setUp() {
        super.setUp();
        command = getInjectorComponent().swerveToNextScoringPositionCommand();
        pose = (MockPoseSubsystem) getInjectorComponent().poseSubsystem();
        drive.getPositionalPid().setP(1);
        drive.getPositionalPid().setI(0);
        drive.getPositionalPid().setD(0);
    }

    @Test
    @Parameters({
            "0, 0, 0, 1",
            "0, 20, 0, 1",
            "0, 80, 3, 4",
            "0, 200, 8, 8",
            "650, 0, 0, 1",
            "650, 20, 0, 1",
            "650, 80, 3, 4",
            "650, 200, 8, 8"
    })
    public void findNextPositionForBlueAllianceLeftTest(double currentX, double currentY, int expectedNearestPositionIndex, int expectedNextPositionIndex) {
        List<Pose2d> scoringPoses = SwerveToNextScoringPositionCommand.getScoringPositionPoses(DriverStation.Alliance.Blue);

        Pose2d nearestPose = SwerveToNextScoringPositionCommand.findNearestScoringPosition(
                new Pose2d(new Translation2d(currentX, currentY), new Rotation2d(0)), DriverStation.Alliance.Blue);

        assertEquals(scoringPoses.get(expectedNearestPositionIndex), nearestPose);
        assertEquals(Rotation2d.fromDegrees(-180), nearestPose.getRotation());
        assertTrue(nearestPose.getX() < 325);

        Pose2d nextPose = SwerveToNextScoringPositionCommand.findNextScoringPosition(
                new Pose2d(new Translation2d(currentX, currentY), new Rotation2d(0)),
                DriverStation.Alliance.Blue,
                SwerveToNextScoringPositionCommand.TargetDirection.Left);

        assertEquals(scoringPoses.get(expectedNextPositionIndex), nextPose);
        assertEquals(Rotation2d.fromDegrees(-180), nextPose.getRotation());
        assertTrue(nextPose.getX() < 325);
    }

    @Test
    @Parameters({
            "0, 0, 0, 0",
            "0, 20, 0, 0",
            "0, 80, 3, 2",
            "0, 200, 8, 7",
            "650, 0, 0, 0",
            "650, 20, 0, 0",
            "650, 80, 3, 2",
            "650, 200, 8, 7"
    })

    public void findNextPositionForBlueAllianceRightTest(double currentX, double currentY, int expectedNearestPositionIndex, int expectedNextPositionIndex) {
        List<Pose2d> scoringPoses = SwerveToNextScoringPositionCommand.getScoringPositionPoses(DriverStation.Alliance.Blue);

        Pose2d nearestPose = SwerveToNextScoringPositionCommand.findNearestScoringPosition(
                new Pose2d(new Translation2d(currentX, currentY), new Rotation2d(0)), DriverStation.Alliance.Blue);

        assertEquals(scoringPoses.get(expectedNearestPositionIndex), nearestPose);
        assertEquals(Rotation2d.fromDegrees(-180), nearestPose.getRotation());
        assertTrue(nearestPose.getX() < 325);

        Pose2d nextPose = SwerveToNextScoringPositionCommand.findNextScoringPosition(
                new Pose2d(new Translation2d(currentX, currentY), new Rotation2d(0)),
                DriverStation.Alliance.Blue,
                SwerveToNextScoringPositionCommand.TargetDirection.Right);

        assertEquals(scoringPoses.get(expectedNextPositionIndex), nextPose);
        assertEquals(Rotation2d.fromDegrees(-180), nextPose.getRotation());
        assertTrue(nextPose.getX() < 325);
    }

    @Test
    @Parameters({
            "0, 0, 0, 0",
            "0, 20, 0, 0",
            "0, 80, 3, 2",
            "0, 200, 8, 7",
            "650, 0, 0, 0",
            "650, 20, 0, 0",
            "650, 80, 3, 2",
            "650, 200, 8, 7"
    })

    public void findNextPositionForRedAllianceLeftTest(double currentX, double currentY, int expectedNearestPositionIndex, int expectedNextPositionIndex) {
        List<Pose2d> scoringPoses = SwerveToNextScoringPositionCommand.getScoringPositionPoses(DriverStation.Alliance.Red);

        Pose2d nearestPose = SwerveToNextScoringPositionCommand.findNearestScoringPosition(
                new Pose2d(new Translation2d(currentX, currentY), new Rotation2d(0)), DriverStation.Alliance.Red);

        assertEquals(scoringPoses.get(expectedNearestPositionIndex), nearestPose);
        assertEquals(Rotation2d.fromDegrees(0), nearestPose.getRotation());
        assertTrue(nearestPose.getX() > 325);

        Pose2d nextPose = SwerveToNextScoringPositionCommand.findNextScoringPosition(
                new Pose2d(new Translation2d(currentX, currentY), new Rotation2d(0)),
                DriverStation.Alliance.Red,
                SwerveToNextScoringPositionCommand.TargetDirection.Left);

        assertEquals(scoringPoses.get(expectedNextPositionIndex), nextPose);
        assertEquals(Rotation2d.fromDegrees(0), nextPose.getRotation());
        assertTrue(nextPose.getX() > 325);
    }

    @Test
    @Parameters({
            "0, 0, 0, 1",
            "0, 20, 0, 1",
            "0, 80, 3, 4",
            "0, 200, 8, 8",
            "650, 0, 0, 1",
            "650, 20, 0, 1",
            "650, 80, 3, 4",
            "650, 200, 8, 8"
    })

    public void findNextPositionForRedAllianceRightTest(double currentX, double currentY, int expectedNearestPositionIndex, int expectedNextPositionIndex) {
        List<Pose2d> scoringPoses = SwerveToNextScoringPositionCommand.getScoringPositionPoses(DriverStation.Alliance.Red);

        Pose2d nearestPose = SwerveToNextScoringPositionCommand.findNearestScoringPosition(
                new Pose2d(new Translation2d(currentX, currentY), new Rotation2d(0)), DriverStation.Alliance.Red);

        assertEquals(scoringPoses.get(expectedNearestPositionIndex), nearestPose);
        assertEquals(Rotation2d.fromDegrees(0), nearestPose.getRotation());
        assertTrue(nearestPose.getX() > 325);

        Pose2d nextPose = SwerveToNextScoringPositionCommand.findNextScoringPosition(
                new Pose2d(new Translation2d(currentX, currentY), new Rotation2d(0)),
                DriverStation.Alliance.Red,
                SwerveToNextScoringPositionCommand.TargetDirection.Right);

        assertEquals(scoringPoses.get(expectedNextPositionIndex), nextPose);
        assertEquals(Rotation2d.fromDegrees(0), nextPose.getRotation());
        assertTrue(nextPose.getX() > 325);
    }

}
