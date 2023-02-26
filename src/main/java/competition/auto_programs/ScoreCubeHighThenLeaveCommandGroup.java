
package competition.auto_programs;

import competition.commandgroups.ScoreGamepieceCommandGroup;
import competition.subsystems.arm.UnifiedArmSubsystem;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.drive.commands.SwerveToPointCommand;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import xbot.common.math.XYPair;

import javax.inject.Inject;

public class ScoreCubeHighThenLeaveCommandGroup extends SequentialCommandGroup {
    @Inject
    ScoreCubeHighThenLeaveCommandGroup(ScoreGamepieceCommandGroup scoreGamepieceCommandGroup,
                                       UnifiedArmSubsystem arm,
                                       SwerveToPointCommand moveOutOfCommunity,
                                       DriveSubsystem drive,
                                       PoseSubsystem pose) {

        scoreGamepieceCommandGroup.setGamePieceModeSupplier(() -> UnifiedArmSubsystem.GamePieceMode.Cube);
        scoreGamepieceCommandGroup.setArmPositionSupplier(() -> UnifiedArmSubsystem.KeyArmPosition.HighGoal);
        scoreGamepieceCommandGroup.setRobotFacingSupplier(() -> UnifiedArmSubsystem.RobotFacing.Backward);

        this.addCommands(scoreGamepieceCommandGroup);

        //drive out of community zone
        moveOutOfCommunity.setFieldRelativeMotion();
        moveOutOfCommunity.setMaxPower(0.5);

        // Our target has to change if we are on red vs blue
        moveOutOfCommunity.setTargetSupplier(
                () -> {
                    var wpiXY = AutoLandmarks.convertBlueToRedIfNeeded(
                            AutoLandmarks.blueNorthOfChargingStationOutsideCommunity
                    ).getTranslation();
                    return new XYPair(wpiXY.getX(), wpiXY.getY());
                },
                () -> pose.rotateAngleBasedOnAlliance(Rotation2d.fromDegrees(0)).getDegrees()
        );

        var driveOut = new ParallelRaceGroup(
                moveOutOfCommunity,
                new WaitCommand(3));
        this.addCommands(driveOut);

        //stop robot
        this.addCommands(new InstantCommand(() -> drive.stop()));
    }
}
