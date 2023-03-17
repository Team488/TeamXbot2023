package competition.auto_programs;

import competition.commandgroups.CollectionSequenceCommandGroup;
import competition.commandgroups.ScoreCubeHighCommandGroup;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.drive.commands.SwerveToPointCommand;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import xbot.common.math.XYPair;

import javax.inject.Inject;

public class ScoreCubeHighThenScoreCubeMidProgram extends SequentialCommandGroup {
    @Inject
    ScoreCubeHighThenScoreCubeMidProgram(ScoreCubeHighThenLeaveProgram scoreCubeHigh,
                                         CollectionSequenceCommandGroup collect,
                                         SwerveToPointCommand moveOutCommunity,
                                         DriveSubsystem drive,
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

        //collect game piece
        this.addCommands(collect);

        //score cube mid


    }
}
