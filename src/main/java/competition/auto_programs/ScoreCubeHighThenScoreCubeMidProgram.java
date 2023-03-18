package competition.auto_programs;

import competition.commandgroups.CollectionSequenceCommandGroup;
import competition.commandgroups.MoveCollectedGamepieceToArmCommandGroup;
import competition.commandgroups.ScoreCubeHighCommandGroup;
import competition.commandgroups.ScoreCubeMidCommandGroup;
import competition.subsystems.arm.UnifiedArmSubsystem;
import competition.subsystems.arm.commands.SimpleXZRouterCommand;
import competition.subsystems.claw.ClawGripperMotorSubsystem;
import competition.subsystems.claw.CloseClawCommand;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.drive.commands.SwerveToPointCommand;
import competition.subsystems.drive.commands.TurnLeft180DegreesCommand;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import xbot.common.math.XYPair;

import javax.inject.Inject;
import javax.inject.Provider;

public class ScoreCubeHighThenScoreCubeMidProgram extends SequentialCommandGroup {
    @Inject
    ScoreCubeHighThenScoreCubeMidProgram(ScoreCubeHighThenLeaveProgram scoreCubeHigh,
                                         CollectionSequenceCommandGroup collect,
                                         SwerveToPointCommand moveToGamePiece,
                                         SwerveToPointCommand driveToScoreMid,
                                         ScoreCubeMidCommandGroup scoreCubeMid,
                                         MoveCollectedGamepieceToArmCommandGroup moveGamePieceToClaw,
                                         SimpleXZRouterCommand retractArm,
                                         ClawGripperMotorSubsystem claw,
                                         CloseClawCommand closeClaw,
                                         PoseSubsystem pose) {
        this.addCommands(scoreCubeHigh);
        //retract arm before moving
        retractArm.setKeyPointFromKeyArmPosition(UnifiedArmSubsystem.KeyArmPosition.FullyRetracted,
                UnifiedArmSubsystem.RobotFacing.Forward);
        var retractArmAndCloseClaw = new SequentialCommandGroup(
                new WaitCommand(1),
                new ParallelCommandGroup(retractArm, closeClaw).deadlineWith(new WaitCommand(5))
        );
        this.addCommands(retractArmAndCloseClaw);

        moveToGamePiece.setFieldRelativeMotion();
        moveToGamePiece.setMaxPower(0.5);
        moveToGamePiece.setMaxTurningPower(0.5);
        //move to game piece
        moveToGamePiece.setTargetSupplier(() -> {
            var XY = AutoLandmarks.convertBlueToRedIfNeeded(AutoLandmarks.blueGamePieceUpper).getTranslation();
            return new XYPair(XY.getX(), XY.getY());
        }, () -> pose.rotateAngleBasedOnAlliance(Rotation2d.fromDegrees(0)).getDegrees());

        var driveToGamePieceAndCollect = new SequentialCommandGroup(new ParallelCommandGroup(moveToGamePiece, collect),moveGamePieceToClaw);
        this.addCommands(driveToGamePieceAndCollect);

        //drive to mid scoring position
        driveToScoreMid.setFieldRelativeMotion();
        driveToScoreMid.setMaxPower(0.5);
        driveToScoreMid.setMaxTurningPower(0.5);

        driveToScoreMid.setTargetSupplier(() -> {
            var XY = AutoLandmarks.convertBlueToRedIfNeeded(AutoLandmarks.blueScoringPositionEight).getTranslation();
            return new XYPair(XY.getX(), XY.getY());
        }, () -> pose.rotateAngleBasedOnAlliance(Rotation2d.fromDegrees(-180)).getDegrees());

        //score cube mid then retract arm
        this.addCommands(scoreCubeMid);
        this.addCommands(retractArmAndCloseClaw);


    }
}
