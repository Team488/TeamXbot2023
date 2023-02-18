package competition.auto_programs;

import competition.subsystems.drive.commands.SwerveToPointCommand;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import xbot.common.math.XYPair;

import javax.inject.Inject;
import javax.inject.Provider;

public class RedTopScoringPath extends SequentialCommandGroup{
    @Inject
    RedTopScoringPath(Provider<SwerveToPointCommand> swerveToPointProvider,PoseSubsystem pose, BluetoRedConversion converter, AutoLandmarks landmarks)
    {
        Pose2d redPose = converter.convertBluetoRed(landmarks.blueScoringPositionNine);
        Pose2d tempRedPose = redPose;
        InstantCommand resetPosition = new InstantCommand(() -> pose.setCurrentPosition((tempRedPose.getX()), tempRedPose.getY()));
        this.addCommands(resetPosition);

        Pose2d finalRedPose = redPose;
        InstantCommand setHeading = new InstantCommand(() -> pose.setCurrentHeading(finalRedPose.getRotation().getDegrees()));
        this.addCommands(setHeading);

        var turn180AndGoToGamePiece = swerveToPointProvider.get();
        redPose = converter.convertBluetoRed(landmarks.blueGamePieceUpper);
        turn180AndGoToGamePiece.setFieldRelativeMotion();
        turn180AndGoToGamePiece.setMaxPower(0.5);
        turn180AndGoToGamePiece.setMaxTurningPower(0.5);
        turn180AndGoToGamePiece.setTargetPosition(new XYPair(redPose.getX(),redPose.getY()), redPose.getRotation().getDegrees());

        this.addCommands(turn180AndGoToGamePiece);

        var returnToScoringZone = swerveToPointProvider.get();
        redPose = converter.convertBluetoRed(landmarks.blueScoringPositionEight);
        returnToScoringZone.setFieldRelativeMotion();
        returnToScoringZone.setMaxPower(0.5);
        returnToScoringZone.setMaxTurningPower(0.5);
        returnToScoringZone.setTargetPosition(new XYPair(redPose.getX(), redPose.getY()), redPose.getRotation().getDegrees());

        this.addCommands(returnToScoringZone);

        var turn180AndGoToCommunitySideMidCheckpoint = swerveToPointProvider.get();
        redPose = converter.convertBluetoRed(landmarks.blueUpperCommunitySideMidCheckpoint);
        turn180AndGoToCommunitySideMidCheckpoint.setFieldRelativeMotion();
        turn180AndGoToCommunitySideMidCheckpoint.setMaxPower(0.5);
        turn180AndGoToCommunitySideMidCheckpoint.setMaxTurningPower(0.5);
        turn180AndGoToCommunitySideMidCheckpoint.setTargetPosition(new XYPair(redPose.getX(), redPose.getY()), redPose.getRotation().getDegrees());

        var goFromCommunitySideMidCheckpointToGamePieceSideCheckpoint = swerveToPointProvider.get();
        redPose = converter.convertBluetoRed(landmarks.blueUpperGamePieceSideMidCheckpoint);
        goFromCommunitySideMidCheckpointToGamePieceSideCheckpoint.setFieldRelativeMotion();
        goFromCommunitySideMidCheckpointToGamePieceSideCheckpoint.setMaxPower(0.5);
        goFromCommunitySideMidCheckpointToGamePieceSideCheckpoint.setMaxTurningPower(0.5);
        goFromCommunitySideMidCheckpointToGamePieceSideCheckpoint.setTargetPosition(new XYPair(redPose.getX(), redPose.getY()), redPose.getRotation().getDegrees());

        this.addCommands(turn180AndGoToCommunitySideMidCheckpoint);

        var goFromMidCheckpointToGamePiece = swerveToPointProvider.get();
        redPose = converter.convertBluetoRed(landmarks.blueGamePieceSecondUpper);
        goFromMidCheckpointToGamePiece.setFieldRelativeMotion();
        goFromMidCheckpointToGamePiece.setMaxPower(0.5);
        goFromMidCheckpointToGamePiece.setMaxTurningPower(0.5);
        goFromMidCheckpointToGamePiece.setTargetPosition(new XYPair(redPose.getX(), redPose.getY()), redPose.getRotation().getDegrees());

        this.addCommands(goFromMidCheckpointToGamePiece);

        var goFromGamePieceToMidCheckpoint = swerveToPointProvider.get();
        redPose = converter.convertBluetoRed(landmarks.blueUpperGamePieceSideMidCheckpoint);
        goFromGamePieceToMidCheckpoint.setFieldRelativeMotion();
        goFromGamePieceToMidCheckpoint.setMaxPower(0.5);
        goFromGamePieceToMidCheckpoint.setMaxTurningPower(0.5);
        goFromGamePieceToMidCheckpoint.setTargetPosition(new XYPair(redPose.getX(), redPose.getY()), redPose.getRotation().getDegrees());

        this.addCommands(goFromGamePieceToMidCheckpoint);

        var goFromMidCheckpointToOtherMidCheckpoint = swerveToPointProvider.get();
        redPose = converter.convertBluetoRed(landmarks.blueUpperCommunitySideMidCheckpoint);
        goFromMidCheckpointToOtherMidCheckpoint.setFieldRelativeMotion();
        goFromMidCheckpointToOtherMidCheckpoint.setMaxPower(0.5);
        goFromMidCheckpointToOtherMidCheckpoint.setMaxTurningPower(0.5);
        goFromMidCheckpointToOtherMidCheckpoint.setTargetPosition(new XYPair(redPose.getX(), redPose.getY()), redPose.getRotation().getDegrees());

        this.addCommands(goFromMidCheckpointToOtherMidCheckpoint);

        var finalReturnToScoringZone = swerveToPointProvider.get();
        redPose = converter.convertBluetoRed(landmarks.blueScoringPositionSeven);
        finalReturnToScoringZone.setFieldRelativeMotion();
        finalReturnToScoringZone.setMaxPower(0.5);
        finalReturnToScoringZone.setMaxTurningPower(0.5);
        finalReturnToScoringZone.setTargetPosition(new XYPair(redPose.getX(), redPose.getY()), redPose.getRotation().getDegrees());

        this.addCommands(finalReturnToScoringZone);
    }
}
