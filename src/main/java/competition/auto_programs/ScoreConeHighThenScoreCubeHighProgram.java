package competition.auto_programs;

import competition.commandgroups.CollectionSequenceCommandGroup;
import competition.commandgroups.MoveCollectedGamepieceToArmCommandGroup;
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
                                          CollectionSequenceCommandGroup collect
                                          ){
        //score cone high

        //move to game piece
        moveToGamePiece.setFieldRelativeMotion();
        moveToGamePiece.setMaxPower(0.5);
        moveToGamePiece.setTargetSupplier(()->{
            var XY = AutoLandmarks.convertBlueToRedIfNeeded(AutoLandmarks.blueGamePieceUpper).getTranslation();
                    return new XYPair(XY.getX(), XY.getY());
        }, () -> pose.rotateAngleBasedOnAlliance(Rotation2d.fromDegrees(0)).getDegrees());

        var collectGamePiece = new SequentialCommandGroup(new ParallelCommandGroup(moveToGamePiece,collect), moveGamePieceToClaw);
        this.addCommands(collectGamePiece);
    }
}
