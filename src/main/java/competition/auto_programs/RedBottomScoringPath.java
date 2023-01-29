package competition.auto_programs;

import javax.inject.Inject;
import javax.inject.Provider;

import competition.subsystems.drive.commands.SwerveToPointCommand;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import xbot.common.math.XYPair;

public class RedBottomScoringPath extends SequentialCommandGroup{
    @Inject
    RedBottomScoringPath(Provider<SwerveToPointCommand> swerveToPointProvider,PoseSubsystem pose)
    {
        var turn180AndGoToGamePiece = swerveToPointProvider.get();
        turn180AndGoToGamePiece.setFieldRelativeMotion();
        turn180AndGoToGamePiece.setMaxPower(0.5);
        turn180AndGoToGamePiece.setMaxTurningPower(0.5);
        turn180AndGoToGamePiece.setTargetPosition(new XYPair(389,36), 180);

        this.addCommands(turn180AndGoToGamePiece);

        var returnToScoringZone = swerveToPointProvider.get();
        returnToScoringZone.setFieldRelativeMotion();
        returnToScoringZone.setMaxPower(0.5);
        returnToScoringZone.setMaxTurningPower(0.5);
        returnToScoringZone.setTargetPosition(new XYPair(584,40), 0 );

        this.addCommands(returnToScoringZone);

        var turn180AndGoToMidCheckpoint = swerveToPointProvider.get();
        turn180AndGoToMidCheckpoint.setFieldRelativeMotion();
        turn180AndGoToMidCheckpoint.setMaxPower(0.5);
        turn180AndGoToMidCheckpoint.setMaxTurningPower(0.5);
        turn180AndGoToMidCheckpoint.setTargetPosition(new XYPair(480,24), 180);

        this.addCommands(turn180AndGoToMidCheckpoint);
        
        var goFromMidCheckpointToGamePiece = swerveToPointProvider.get();
        goFromMidCheckpointToGamePiece.setFieldRelativeMotion();
        goFromMidCheckpointToGamePiece.setMaxPower(0.5);
        goFromMidCheckpointToGamePiece.setMaxTurningPower(0.5);
        goFromMidCheckpointToGamePiece.setTargetPosition(new XYPair(389,84), 180);

        this.addCommands(goFromMidCheckpointToGamePiece);

        var goFromGamePieceToMidCheckpoint = swerveToPointProvider.get();
        goFromGamePieceToMidCheckpoint.setFieldRelativeMotion();
        goFromGamePieceToMidCheckpoint.setMaxPower(0.5);
        goFromGamePieceToMidCheckpoint.setMaxTurningPower(0.5);
        goFromGamePieceToMidCheckpoint.setTargetPosition(new XYPair(455,27), 0);

        this.addCommands(goFromGamePieceToMidCheckpoint);

        var goFromMidCheckpointToOtherMidCheckpoint = swerveToPointProvider.get();
        goFromMidCheckpointToOtherMidCheckpoint.setFieldRelativeMotion();
        goFromMidCheckpointToOtherMidCheckpoint.setMaxPower(0.5);
        goFromMidCheckpointToOtherMidCheckpoint.setMaxTurningPower(0.5);
        goFromMidCheckpointToOtherMidCheckpoint.setTargetPosition(new XYPair(527,29), 0);

        this.addCommands(goFromMidCheckpointToOtherMidCheckpoint);

        var finalReturnToScoringZone = swerveToPointProvider.get();
        finalReturnToScoringZone.setFieldRelativeMotion();
        finalReturnToScoringZone.setMaxPower(0.5);
        finalReturnToScoringZone.setMaxTurningPower(0.5);
        finalReturnToScoringZone.setTargetPosition(new XYPair(584,64), 0);

        this.addCommands(finalReturnToScoringZone);
    }
}
