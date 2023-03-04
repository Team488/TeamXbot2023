
package competition.auto_programs;

import competition.commandgroups.ScoreCubeHighCommandGroup;
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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import xbot.common.math.XYPair;

import javax.inject.Inject;

public class ScoreCubeHighThenLeaveHighProgram extends SequentialCommandGroup {
    @Inject
    ScoreCubeHighThenLeaveHighProgram(ScoreCubeHighCommandGroup scoreCubeHighCommandGroup,
                                      SimpleXZRouterCommand retractArm,
                                      CloseClawCommand closeClaw,
                                      SwerveToPointCommand moveOutOfCommunity,
                                      DriveSubsystem drive,
                                      PoseSubsystem pose) {

        // Set pose to specific location (eventually, ask the camera if it already has a good idea of where we are)
        // We will start facing away from the driver station.
        var forcePosition = new InstantCommand(
                () -> {
                    var translation =
                            AutoLandmarks.convertBlueToRedIfNeeded(AutoLandmarks.blueScoringPositionEight)
                                    .getTranslation().times(1.0 / PoseSubsystem.INCHES_IN_A_METER);
                    var rotation = pose.rotateAngleBasedOnAlliance(Rotation2d.fromDegrees(-180));
                    pose.setCurrentPoseInMeters(new Pose2d(translation, rotation));
                }
        );
        this.addCommands(forcePosition);

        this.addCommands(scoreCubeHighCommandGroup);

        // -------------------------
        // Drive out of community zone while retracting arm and closing claw
        // -------------------------

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
                () -> pose.rotateAngleBasedOnAlliance(Rotation2d.fromDegrees(-180)).getDegrees()
        );

        retractArm.setKeyPointFromKeyArmPosition(
                UnifiedArmSubsystem.KeyArmPosition.PrepareToAcquireFromCollector,
                UnifiedArmSubsystem.RobotFacing.Forward);

        var waitThenRetractArmAndCloseClaw = new SequentialCommandGroup(
                new WaitCommand(1),
                new ParallelCommandGroup(
                        retractArm,
                        closeClaw).deadlineWith(new WaitCommand(5))
        );

        var driveOutWhileResettingArm = new ParallelCommandGroup(
                moveOutOfCommunity,
                waitThenRetractArmAndCloseClaw);

        this.addCommands(driveOutWhileResettingArm);

        //stop robot
        this.addCommands(new InstantCommand(() -> drive.stop()));
    }
}
