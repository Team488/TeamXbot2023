package competition.auto_programs;

import competition.commandgroups.ScoreConeHighCommandGroup;
import competition.subsystems.arm.UnifiedArmSubsystem;
import competition.subsystems.arm.commands.SimpleXZRouterCommand;
import competition.subsystems.claw.CloseClawCommand;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.drive.commands.SwerveToPointCommand;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import xbot.common.math.XYPair;

import javax.inject.Inject;

public class ScoreConeHighThenLeave extends SequentialCommandGroup {
    @Inject
    ScoreConeHighThenLeave(SimpleXZRouterCommand retractArm,
                           SwerveToPointCommand moveOutOfCommunity,
                           ScoreConeHighCommandGroup scoreConeHigh,
                           DriveSubsystem drive,
                           PoseSubsystem pose){
        var forcePosition = new InstantCommand(() -> {
            var translation = AutoLandmarks.convertBlueToRedIfNeeded(AutoLandmarks.blueScoringPositionNine)
                            .getTranslation().times(1/PoseSubsystem.INCHES_IN_A_METER);
            var rotation = pose.rotateAngleBasedOnAlliance(Rotation2d.fromDegrees(-180));
            pose.setCurrentPoseInMeters(new Pose2d(translation, rotation));
        });

        this.addCommands(forcePosition);

        //score cone high
        this.addCommands(scoreConeHigh);
        //drive out of community
        moveOutOfCommunity.setFieldRelativeMotion();
        moveOutOfCommunity.setMaxPower(0.5);

        moveOutOfCommunity.setTargetSupplier(() -> {
            var wpiXY = AutoLandmarks.convertBlueToRedIfNeeded(AutoLandmarks.blueNorthOfChargingStationOutsideCommunity).getTranslation();
            return new XYPair(wpiXY.getX(),wpiXY.getY());
        }, () -> pose.rotateAngleBasedOnAlliance(Rotation2d.fromDegrees(-180)).getDegrees());

        retractArm.setKeyPointFromKeyArmPosition(
                UnifiedArmSubsystem.KeyArmPosition.FullyRetracted,
                UnifiedArmSubsystem.RobotFacing.Forward);

        var waitThenRetractArm = new ParallelCommandGroup(new WaitCommand(1), retractArm);
        var driveOutWhileRetractingArm = new ParallelCommandGroup(moveOutOfCommunity,waitThenRetractArm);
        this.addCommands(driveOutWhileRetractingArm);

        this.addCommands(new InstantCommand(() -> drive.stop()));
    }
}
