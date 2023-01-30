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
    BlueTopScoringPath(Provider<SwerveToPointCommand> swerveToPointProvider,PoseSubsystem pose)
    {
        InstantCommand resetPosition = new InstantCommand(() -> pose.setCurrentPosition(66, 200));
        this.addCommands(resetPosition);

        InstantCommand setHeading = new InstantCommand(() -> pose.setCurrentHeading(-180));
        this.addCommands(setHeading);

        var turn180AndGoToGamePiece = swerveToPointProvider.get();
        turn180AndGoToGamePiece.setFieldRelativeMotion();
        turn180AndGoToGamePiece.setMaxPower(0.5);
        turn180AndGoToGamePiece.setMaxTurningPower(0.5);
        turn180AndGoToGamePiece.setTargetPosition(new XYPair(257,182), 0);

        this.addCommands(turn180AndGoToGamePiece);

        var returnToScoringZone = swerveToPointProvider.get();
        returnToScoringZone.setFieldRelativeMotion();
        returnToScoringZone.setMaxPower(0.5);
        returnToScoringZone.setMaxTurningPower(0.5);
        returnToScoringZone.setTargetPosition(new XYPair(66,175), 180);

        this.addCommands(returnToScoringZone);

        var turn180AndGoToMidCheckpoint = swerveToPointProvider.get();
        turn180AndGoToMidCheckpoint.setFieldRelativeMotion();
        turn180AndGoToMidCheckpoint.setMaxPower(0.5);
        turn180AndGoToMidCheckpoint.setMaxTurningPower(0.5);
        turn180AndGoToMidCheckpoint.setTargetPosition(new XYPair(195,183), 0);

        this.addCommands(turn180AndGoToMidCheckpoint);
        
        var goFromMidCheckpointToGamePiece = swerveToPointProvider.get();
        goFromMidCheckpointToGamePiece.setFieldRelativeMotion();
        goFromMidCheckpointToGamePiece.setMaxPower(0.5);
        goFromMidCheckpointToGamePiece.setMaxTurningPower(0.5);
        goFromMidCheckpointToGamePiece.setTargetPosition(new XYPair(261,131), 0);

        this.addCommands(goFromMidCheckpointToGamePiece);

        var goFromGamePieceToMidCheckpoint = swerveToPointProvider.get();
        goFromGamePieceToMidCheckpoint.setFieldRelativeMotion();
        goFromGamePieceToMidCheckpoint.setMaxPower(0.5);
        goFromGamePieceToMidCheckpoint.setMaxTurningPower(0.5);
        goFromGamePieceToMidCheckpoint.setTargetPosition(new XYPair(261,177), 180);

        this.addCommands(goFromGamePieceToMidCheckpoint);

        var goFromMidCheckpointToOtherMidCheckpoint = swerveToPointProvider.get();
        goFromMidCheckpointToOtherMidCheckpoint.setFieldRelativeMotion();
        goFromMidCheckpointToOtherMidCheckpoint.setMaxPower(0.5);
        goFromMidCheckpointToOtherMidCheckpoint.setMaxTurningPower(0.5);
        goFromMidCheckpointToOtherMidCheckpoint.setTargetPosition(new XYPair(118,183), 180);

        this.addCommands(goFromMidCheckpointToOtherMidCheckpoint);

        var finalReturnToScoringZone = swerveToPointProvider.get();
        finalReturnToScoringZone.setFieldRelativeMotion();
        finalReturnToScoringZone.setMaxPower(0.5);
        finalReturnToScoringZone.setMaxTurningPower(0.5);
        finalReturnToScoringZone.setTargetPosition(new XYPair(66,152), 180);

        this.addCommands(finalReturnToScoringZone);
    }
}
