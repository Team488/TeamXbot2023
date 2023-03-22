package competition.auto_programs;

import competition.commandgroups.CollectionSequenceCommandGroup;
import competition.commandgroups.MoveCollectedGamepieceToArmCommandGroup;
import competition.commandgroups.ScoreCubeHighCommandGroup;
import competition.subsystems.arm.UnifiedArmSubsystem;
import competition.subsystems.arm.commands.SimpleXZRouterCommand;
import competition.subsystems.claw.CloseClawCommand;
import competition.subsystems.drive.commands.SwerveToPointCommand;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import xbot.common.math.XYPair;

public class ScoreConeHighThenScoreCubeHighProgram extends SequentialCommandGroup {
    ScoreConeHighThenScoreCubeHighProgram(MoveCollectedGamepieceToArmCommandGroup moveGamePieceToClaw,
                                          SimpleXZRouterCommand retractArm,
                                          CloseClawCommand closeClaw,
                                          PoseSubsystem pose,
                                          SwerveToPointCommand moveToGamePiece,
                                          SwerveToPointCommand moveToScore,
                                          CollectionSequenceCommandGroup collect,
                                          ScoreCubeHighCommandGroup scoreCubeHigh,
                                          ScoreConeHighThenLeave scoreConeHighThenLeave
                                          ){
        //score cone high
        this.addCommands(scoreConeHighThenLeave);
        //move to game piece
        moveToGamePiece.setFieldRelativeMotion();
        moveToGamePiece.setMaxPower(0.5);
        moveToGamePiece.setMaxTurningPower(0.5);
        moveToGamePiece.setTargetSupplier(()->{
            var gamePieceXY = AutoLandmarks.convertBlueToRedIfNeeded(AutoLandmarks.blueGamePieceUpper).getTranslation();
                    return new XYPair(gamePieceXY.getX(), gamePieceXY.getY());
        }, () -> pose.rotateAngleBasedOnAlliance(Rotation2d.fromDegrees(0)).getDegrees());

        var collectGamePiece = new SequentialCommandGroup(new ParallelCommandGroup(moveToGamePiece,collect), moveGamePieceToClaw);
        this.addCommands(collectGamePiece);

        //move to scoring position
        moveToScore.setFieldRelativeMotion();
        moveToScore.setMaxPower(0.5);
        moveToScore.setMaxTurningPower(0.5);

        moveToScore.setTargetSupplier(() -> {
            var targetXY = AutoLandmarks.convertBlueToRedIfNeeded(AutoLandmarks.blueScoringPositionEight).getTranslation();
            return new XYPair(targetXY.getX(),targetXY.getY());
        }, () -> pose.rotateAngleBasedOnAlliance(Rotation2d.fromDegrees(-180)).getDegrees());

        //score cube high
        var driveToScoreCube = new ParallelCommandGroup(moveToScore,scoreCubeHigh);
        this.addCommands(driveToScoreCube);

        //retract arm and close claw
        retractArm.setKeyPointFromKeyArmPosition(UnifiedArmSubsystem.KeyArmPosition.FullyRetracted, UnifiedArmSubsystem.RobotFacing.Forward);
        var retractArmAndClaw = new ParallelCommandGroup(retractArm, closeClaw);
        this.addCommands(retractArmAndClaw);
    }
}
