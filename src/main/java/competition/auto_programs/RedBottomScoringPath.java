package competition.auto_programs;

import competition.subsystems.drive.commands.SwerveToPointCommand;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import xbot.common.math.XYPair;

import javax.inject.Inject;
import javax.inject.Provider;

public class RedBottomScoringPath extends SequentialCommandGroup{
    @Inject
    RedBottomScoringPath(Provider<SwerveToPointCommand> swerveToPointProvider,PoseSubsystem pose, BluetoRedConversion converter, AutoLandmarks landmarks)
    {
        Pose2d redPose = converter.convertBluetoRed(landmarks.blueScoringPositionOne);
        Pose2d tempRedPose = redPose;
        InstantCommand resetPosition = new InstantCommand(() -> pose.setCurrentPosition((tempRedPose.getX()), tempRedPose.getY()));
        this.addCommands(resetPosition);

        Pose2d finalRedPose = redPose;
        InstantCommand setHeading = new InstantCommand(() -> pose.setCurrentHeading(finalRedPose.getRotation().getDegrees()));
        this.addCommands(setHeading);

        var turn180AndGoToGamePiece = swerveToPointProvider.get();
        redPose = converter.convertBluetoRed(landmarks.blueGamePieceLower);
        turn180AndGoToGamePiece.setFieldRelativeMotion();
        turn180AndGoToGamePiece.setMaxPower(0.5);
        turn180AndGoToGamePiece.setMaxTurningPower(0.5);
        turn180AndGoToGamePiece.setTargetPosition(new XYPair(redPose.getX(),redPose.getY()), redPose.getRotation().getDegrees());

        this.addCommands(turn180AndGoToGamePiece);

        var returnToScoringZone = swerveToPointProvider.get();
        redPose = converter.convertBluetoRed(landmarks.blueScoringPositionTwo);
        returnToScoringZone.setFieldRelativeMotion();
        returnToScoringZone.setMaxPower(0.5);
        returnToScoringZone.setMaxTurningPower(0.5);
        returnToScoringZone.setTargetPosition(new XYPair(redPose.getX(), redPose.getY()), redPose.getRotation().getDegrees());

        this.addCommands(returnToScoringZone);

        var turn180AndGoToCommunitySideMidCheckpoint = swerveToPointProvider.get();
        redPose = converter.convertBluetoRed(landmarks.blueLowerCommunitySideMidCheckpoint);
        turn180AndGoToCommunitySideMidCheckpoint.setFieldRelativeMotion();
        turn180AndGoToCommunitySideMidCheckpoint.setMaxPower(0.5);
        turn180AndGoToCommunitySideMidCheckpoint.setMaxTurningPower(0.5);
        turn180AndGoToCommunitySideMidCheckpoint.setTargetPosition(
                new XYPair(redPose.getX(), redPose.getY()), redPose.getRotation().getDegrees());

        var goFromCommunitySideMidCheckpointToGamePieceSideCheckpoint = swerveToPointProvider.get();
        redPose = converter.convertBluetoRed(landmarks.blueLowerGamePieceSideMidCheckpoint);
        goFromCommunitySideMidCheckpointToGamePieceSideCheckpoint.setFieldRelativeMotion();
        goFromCommunitySideMidCheckpointToGamePieceSideCheckpoint.setMaxPower(0.5);
        goFromCommunitySideMidCheckpointToGamePieceSideCheckpoint.setMaxTurningPower(0.5);
        goFromCommunitySideMidCheckpointToGamePieceSideCheckpoint.setTargetPosition(
                new XYPair(redPose.getX(), redPose.getY()), redPose.getRotation().getDegrees());

        this.addCommands(turn180AndGoToCommunitySideMidCheckpoint);

        var goFromMidCheckpointToGamePiece = swerveToPointProvider.get();
        redPose = converter.convertBluetoRed(landmarks.blueGamePieceSecondLower);
        goFromMidCheckpointToGamePiece.setFieldRelativeMotion();
        goFromMidCheckpointToGamePiece.setMaxPower(0.5);
        goFromMidCheckpointToGamePiece.setMaxTurningPower(0.5);
        goFromMidCheckpointToGamePiece.setTargetPosition(
                new XYPair(redPose.getX(), redPose.getY()), redPose.getRotation().getDegrees());

        this.addCommands(goFromMidCheckpointToGamePiece);

        var goFromGamePieceToMidCheckpoint = swerveToPointProvider.get();
        redPose = converter.convertBluetoRed(landmarks.blueLowerGamePieceSideMidCheckpoint);
        goFromGamePieceToMidCheckpoint.setFieldRelativeMotion();
        goFromGamePieceToMidCheckpoint.setMaxPower(0.5);
        goFromGamePieceToMidCheckpoint.setMaxTurningPower(0.5);
        goFromGamePieceToMidCheckpoint.setTargetPosition(
                new XYPair(redPose.getX(), redPose.getY()), redPose.getRotation().getDegrees());

        this.addCommands(goFromGamePieceToMidCheckpoint);

        var goFromMidCheckpointToOtherMidCheckpoint = swerveToPointProvider.get();
        redPose = converter.convertBluetoRed(landmarks.blueLowerCommunitySideMidCheckpoint);
        goFromMidCheckpointToOtherMidCheckpoint.setFieldRelativeMotion();
        goFromMidCheckpointToOtherMidCheckpoint.setMaxPower(0.5);
        goFromMidCheckpointToOtherMidCheckpoint.setMaxTurningPower(0.5);
        goFromMidCheckpointToOtherMidCheckpoint.setTargetPosition(
                new XYPair(redPose.getX(), redPose.getY()), redPose.getRotation().getDegrees());

        this.addCommands(goFromMidCheckpointToOtherMidCheckpoint);

        var finalReturnToScoringZone = swerveToPointProvider.get();
        redPose = converter.convertBluetoRed(landmarks.blueScoringPositionThree);
        finalReturnToScoringZone.setFieldRelativeMotion();
        finalReturnToScoringZone.setMaxPower(0.5);
        finalReturnToScoringZone.setMaxTurningPower(0.5);
        finalReturnToScoringZone.setTargetPosition(
                new XYPair(redPose.getX(), redPose.getY()), redPose.getRotation().getDegrees());

        this.addCommands(finalReturnToScoringZone);
    }
}
