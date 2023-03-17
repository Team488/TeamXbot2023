package competition.auto_programs;

import competition.commandgroups.CollectionSequenceCommandGroup;
import competition.commandgroups.ScoreCubeHighCommandGroup;
import competition.commandgroups.ScoreCubeMidCommandGroup;
import competition.subsystems.arm.UnifiedArmSubsystem;
import competition.subsystems.arm.commands.SimpleXZRouterCommand;
import competition.subsystems.claw.CloseClawCommand;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.drive.commands.SwerveToPointCommand;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import xbot.common.math.XYPair;

import javax.inject.Inject;

public class ScoreCubeHighThenScoreCubeMidProgram extends SequentialCommandGroup {
    @Inject
    ScoreCubeHighThenScoreCubeMidProgram(ScoreCubeHighThenLeaveProgram scoreCubeHigh,
                                         CollectionSequenceCommandGroup collect,
                                         SwerveToPointCommand moveOutCommunity,
                                         SwerveToPointCommand turnToCube,
                                         SwerveToPointCommand driveToScoreMid,
                                         ScoreCubeMidCommandGroup scoreCubeMid,
                                         SimpleXZRouterCommand retractArm,
                                         DriveSubsystem drive,
                                         CloseClawCommand closeClaw,
                                         PoseSubsystem pose) {
        this.addCommands(scoreCubeHigh);

        moveOutCommunity.setFieldRelativeMotion();
        moveOutCommunity.setMaxPower(0.5);
        moveOutCommunity.setMaxTurningPower(0.5);
        //move to game piece
        moveOutCommunity.setTargetSupplier(() -> {
            var XY = AutoLandmarks.convertBlueToRedIfNeeded(AutoLandmarks.blueNorthOfChargingStationGamePiece).getTranslation();
            return new XYPair(XY.getX(), XY.getY());
        }, () -> pose.rotateAngleBasedOnAlliance(Rotation2d.fromDegrees(-180)).getDegrees());

        var driveToGamePieceAndCollect = new ParallelCommandGroup(moveOutCommunity, collect);
        this.addCommands(driveToGamePieceAndCollect);

        //drive to mid scoring position
        driveToScoreMid.setFieldRelativeMotion();
        driveToScoreMid.setMaxPower(0.5);
        driveToScoreMid.setMaxTurningPower(0.5);

      /*  driveToScoreMid.setTargetSupplier(() -> {
            var XY = AutoLandmarks.convertBlueToRedIfNeeded(AutoLandmarks.).getTranslation();
            return new XYPair(XY.getX(), XY.getY());
        }, () -> pose.rotateAngleBasedOnAlliance(Rotation2d.fromDegrees(-180)).getDegrees());*/

        retractArm.setKeyPointFromKeyArmPosition(UnifiedArmSubsystem.KeyArmPosition.PrepareToAcquireFromCollector,
                UnifiedArmSubsystem.RobotFacing.Forward);

        //score cube mid then retract arm
        var retractArmAndCloseClaw = new SequentialCommandGroup(
                new WaitCommand(1),
                new ParallelCommandGroup(retractArm, closeClaw).deadlineWith(new WaitCommand(5))
        );




    }
}
