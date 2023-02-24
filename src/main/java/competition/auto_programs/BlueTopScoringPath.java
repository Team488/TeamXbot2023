package competition.auto_programs;

import javax.inject.Inject;
import javax.inject.Provider;

import competition.subsystems.drive.commands.SwerveToPointCommand;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import xbot.common.math.XYPair;

public class BlueTopScoringPath extends SequentialCommandGroup{
    @Inject
    BlueTopScoringPath(Provider<SwerveToPointCommand> swerveToPointProvider,PoseSubsystem pose, AutoLandmarks landmarks)
    {
        InstantCommand resetPosition = new InstantCommand(
                () -> pose.setCurrentPosition(landmarks.blueScoringPositionNine.getX(), landmarks.blueScoringPositionNine.getY()));
        this.addCommands(resetPosition);

        InstantCommand setHeading = new InstantCommand(() -> pose.setCurrentHeading(-180));
        this.addCommands(setHeading);

        var turn180AndGoToGamePiece = swerveToPointProvider.get();
        turn180AndGoToGamePiece.setFieldRelativeMotion();
        turn180AndGoToGamePiece.setMaxPower(0.5);
        turn180AndGoToGamePiece.setMaxTurningPower(0.5);
        turn180AndGoToGamePiece.setTargetPosition(
                new XYPair(landmarks.blueGamePieceUpper.getX(), landmarks.blueGamePieceUpper.getY()),
                landmarks.blueGamePieceUpper.getRotation().getDegrees());

        this.addCommands(turn180AndGoToGamePiece);

        var returnToScoringZone = swerveToPointProvider.get();
        returnToScoringZone.setFieldRelativeMotion();
        returnToScoringZone.setMaxPower(0.5);
        returnToScoringZone.setMaxTurningPower(0.5);
        returnToScoringZone.setTargetPosition(
                new XYPair(landmarks.blueScoringPositionEight.getX(), landmarks.blueScoringPositionEight.getY()),
                landmarks.blueScoringPositionEight.getRotation().getDegrees());

        this.addCommands(returnToScoringZone);

        var turn180AndGoToCommunitySideMidCheckpoint = swerveToPointProvider.get();
        turn180AndGoToCommunitySideMidCheckpoint.setFieldRelativeMotion();
        turn180AndGoToCommunitySideMidCheckpoint.setMaxPower(0.5);
        turn180AndGoToCommunitySideMidCheckpoint.setMaxTurningPower(0.5);
        turn180AndGoToCommunitySideMidCheckpoint.setTargetPosition(
                new XYPair(landmarks.blueUpperCommunitySideMidCheckpoint.getX(),landmarks.blueUpperCommunitySideMidCheckpoint.getY()),
                landmarks.blueUpperCommunitySideMidCheckpoint.getRotation().getDegrees());

        this.addCommands(turn180AndGoToCommunitySideMidCheckpoint);

        var goFromCommunitySideMidCheckpointToGamePieceSideCheckpoint = swerveToPointProvider.get();
        goFromCommunitySideMidCheckpointToGamePieceSideCheckpoint.setFieldRelativeMotion();
        goFromCommunitySideMidCheckpointToGamePieceSideCheckpoint.setMaxPower(0.5);
        goFromCommunitySideMidCheckpointToGamePieceSideCheckpoint.setMaxTurningPower(0.5);
        goFromCommunitySideMidCheckpointToGamePieceSideCheckpoint.setTargetPosition(
                new XYPair(landmarks.blueUpperGamePieceSideMidCheckpoint.getX(),landmarks.blueUpperGamePieceSideMidCheckpoint.getY()),
                landmarks.blueUpperGamePieceSideMidCheckpoint.getRotation().getDegrees());

        var goFromMidCheckpointToGamePiece = swerveToPointProvider.get();
        goFromMidCheckpointToGamePiece.setFieldRelativeMotion();
        goFromMidCheckpointToGamePiece.setMaxPower(0.5);
        goFromMidCheckpointToGamePiece.setMaxTurningPower(0.5);
        goFromMidCheckpointToGamePiece.setTargetPosition(
                new XYPair(landmarks.blueGamePieceSecondUpper.getX(),landmarks.blueGamePieceSecondUpper.getY()),
                landmarks.blueGamePieceSecondUpper.getRotation().getDegrees());

        this.addCommands(goFromMidCheckpointToGamePiece);

        var goFromGamePieceToMidCheckpoint = swerveToPointProvider.get();
        goFromGamePieceToMidCheckpoint.setFieldRelativeMotion();
        goFromGamePieceToMidCheckpoint.setMaxPower(0.5);
        goFromGamePieceToMidCheckpoint.setMaxTurningPower(0.5);
        goFromGamePieceToMidCheckpoint.setTargetPosition(
                new XYPair(landmarks.blueUpperGamePieceSideMidCheckpoint.getX(),landmarks.blueUpperGamePieceSideMidCheckpoint.getY()),
                landmarks.blueUpperGamePieceSideMidCheckpoint.getRotation().getDegrees());

        this.addCommands(goFromGamePieceToMidCheckpoint);

        var goFromMidCheckpointToOtherMidCheckpoint = swerveToPointProvider.get();
        goFromMidCheckpointToOtherMidCheckpoint.setFieldRelativeMotion();
        goFromMidCheckpointToOtherMidCheckpoint.setMaxPower(0.5);
        goFromMidCheckpointToOtherMidCheckpoint.setMaxTurningPower(0.5);
        goFromMidCheckpointToOtherMidCheckpoint.setTargetPosition(
                new XYPair(landmarks.blueUpperCommunitySideMidCheckpoint.getX(),landmarks.blueUpperCommunitySideMidCheckpoint.getY()),
                landmarks.blueUpperCommunitySideMidCheckpoint.getRotation().getDegrees());

        this.addCommands(goFromMidCheckpointToOtherMidCheckpoint);

        var finalReturnToScoringZone = swerveToPointProvider.get();
        finalReturnToScoringZone.setFieldRelativeMotion();
        finalReturnToScoringZone.setMaxPower(0.5);
        finalReturnToScoringZone.setMaxTurningPower(0.5);
        finalReturnToScoringZone.setTargetPosition(
                new XYPair(landmarks.blueScoringPositionSeven.getX(),landmarks.blueScoringPositionSeven.getY()),
                landmarks.blueScoringPositionSeven.getRotation().getDegrees());

        this.addCommands(finalReturnToScoringZone);
    }
}
