package competition.auto_programs;

import competition.commandgroups.ScoreCubeHighCommandGroup;
import competition.subsystems.arm.UnifiedArmSubsystem;
import competition.subsystems.arm.commands.SimpleXZRouterCommand;
import competition.subsystems.claw.CloseClawCommand;
import competition.subsystems.drive.commands.AutoBalanceCommand;
import competition.subsystems.drive.commands.BrakeCommand;
import competition.subsystems.drive.commands.SwerveSimpleTrajectoryCommand;
import competition.subsystems.drive.commands.VelocityMaintainerCommand;
import competition.subsystems.drive.commands.XbotSwervePoint;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import javax.inject.Inject;
import java.util.ArrayList;
import java.util.List;

public class ScoreCubeHighThenBalanceProgram extends SequentialCommandGroup {

    @Inject
    public ScoreCubeHighThenBalanceProgram(
            PoseSubsystem pose,
            ScoreCubeHighCommandGroup scoreCubeHigh,
            SimpleXZRouterCommand retractArm,
            CloseClawCommand closeClaw,
            SwerveSimpleTrajectoryCommand mantleChargePlate,
            AutoBalanceCommand autoBalance,
            VelocityMaintainerCommand velocityMaintainer,
            BrakeCommand brake) {

        // Force set our current position - facing the grid station, ready to score our cone.
        var forcePosition = new InstantCommand(
                () -> {
                    var translation =
                            AutoLandmarks.convertBlueToRedIfNeeded(AutoLandmarks.blueScoringPositionFive)
                                    .getTranslation().times(1.0 / PoseSubsystem.INCHES_IN_A_METER);
                    var rotation = pose.rotateAngleBasedOnAlliance(Rotation2d.fromDegrees(-180));
                    pose.setCurrentPoseInMeters(new Pose2d(translation, rotation));
                }
        );
        this.addCommands(forcePosition);

        // Score high
        this.addCommands(scoreCubeHigh);

        // Get the arm back safe
        retractArm.setKeyPointFromKeyArmPosition(
                UnifiedArmSubsystem.KeyArmPosition.PrepareToAcquireFromCollector,
                UnifiedArmSubsystem.RobotFacing.Forward);

        var retractArmAndCloseClaw = new ParallelCommandGroup(
                        retractArm,
                        new WaitCommand(1.0).andThen(closeClaw)
        ).withTimeout(2.0);

        this.addCommands(retractArmAndCloseClaw);

        mantleChargePlate.setMaxPower(1.0);
        mantleChargePlate.setMaxTurningPower(0.5);
        mantleChargePlate.setKeyPointsProvider(
                () -> {
                    // Get on that charge station over 1 seconds
                    var chargeStationMantlePoint =
                            AutoLandmarks.convertBlueToRedIfNeeded(AutoLandmarks.blueChargeStationCenter);
                    XbotSwervePoint getOnChargeStation = new XbotSwervePoint(
                            chargeStationMantlePoint.getX(),
                            chargeStationMantlePoint.getY(),
                            pose.rotateAngleBasedOnAlliance(Rotation2d.fromDegrees(-180)).getDegrees(),
                            1.0);
                    return new ArrayList<>(List.of(getOnChargeStation));
                });
        // This is only supposed to take 2.5 seconds. Set a timeout just in case.
        this.addCommands(mantleChargePlate.withTimeout(3.0));

        // With a 15-second budget, and subtracting 1 second for scoring, and 3 seconds for mantle, we have 11 seconds to balance.
        // to make 100% sure we engage the brakes, set a timeout for just under that.
        var balance = new ParallelRaceGroup(
                autoBalance,
                velocityMaintainer,
                new WaitCommand(10.75)
        );
        this.addCommands(balance);
        this.addCommands(brake);
    }
}