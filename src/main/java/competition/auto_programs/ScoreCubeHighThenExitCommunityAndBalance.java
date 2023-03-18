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

public class ScoreCubeHighThenExitCommunityAndBalance extends SequentialCommandGroup{
    @Inject
    public ScoreCubeHighThenExitCommunityAndBalance(
            PoseSubsystem pose,
            ScoreCubeHighCommandGroup scoreCubeHigh,
            SimpleXZRouterCommand retractArm,
            CloseClawCommand closeClaw,
            SwerveSimpleTrajectoryCommand turnTowardsField,
            SwerveSimpleTrajectoryCommand exitCommunity,
            SwerveSimpleTrajectoryCommand turnTowardsCommunity,
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

        turnTowardsField.setMaxPower(1.0);
        turnTowardsField.setMaxTurningPower(0.5);
        turnTowardsField.setKeyPointsProvider(
                () -> {
                    // Turn around while backing up to prepare for mantling
                    var backOffFromScoringPositionsAndTurnAround =
                            AutoLandmarks.convertBlueToRedIfNeeded(AutoLandmarks.blueChargeStationMantleFromLeft);
                    XbotSwervePoint backOffAndTurnAround = new XbotSwervePoint(
                            backOffFromScoringPositionsAndTurnAround.getX(),
                            backOffFromScoringPositionsAndTurnAround.getY(),
                            pose.rotateAngleBasedOnAlliance(Rotation2d.fromDegrees(0)).getDegrees(),
                            1.0);
                    return new ArrayList<>(List.of(backOffAndTurnAround));
                });
        this.addCommands(turnTowardsField);

        exitCommunity.setMaxPower(1.0);
        exitCommunity.setMaxTurningPower(0.5);
        exitCommunity.setKeyPointsProvider(
                () -> {
                    // Get over that charge station in (maybe?) 2 seconds
                    var mantleChargeStationAndExitCommunity =
                            AutoLandmarks.convertBlueToRedIfNeeded(AutoLandmarks.blueChargeStationMantleFromRight);
                    XbotSwervePoint getOnChargeStationAndExit = new XbotSwervePoint(
                            mantleChargeStationAndExitCommunity.getX(),
                            mantleChargeStationAndExitCommunity.getY(),
                            pose.rotateAngleBasedOnAlliance(Rotation2d.fromDegrees(0)).getDegrees(),
                            2.0);
                    return new ArrayList<>(List.of(getOnChargeStationAndExit));
                });
        this.addCommands(exitCommunity);


        turnTowardsCommunity.setMaxPower(1.0);
        turnTowardsCommunity.setMaxTurningPower(0.5);
        turnTowardsCommunity.setKeyPointsProvider(
                () -> {
                    // Turn around in place from the right side of charge station to prepare for mantling
                    var turnAroundInPlace =
                            AutoLandmarks.convertBlueToRedIfNeeded(AutoLandmarks.blueChargeStationMantleFromRight);
                    XbotSwervePoint turnAround = new XbotSwervePoint(
                            turnAroundInPlace.getX(),
                            turnAroundInPlace.getY(),
                            pose.rotateAngleBasedOnAlliance(Rotation2d.fromDegrees(-180)).getDegrees(),
                            1.0);
                    return new ArrayList<>(List.of(turnAround));
                });
        this.addCommands(turnTowardsCommunity);

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

        // (Slight edit) With a 15-second budget, and subtracting 1 second for scoring,
        // * 2 seconds to get over charge station and exit community*,
        // * 2 total seconds to turn the robot around and prepare for the climb*,
        // and 3 seconds for mantle, we have 7 seconds to balance.
        // to make 100% sure we engage the brakes, set a timeout for just under that.

        // 2 seconds is completely pulled out of thin air, not sure if it'll need more time.
        var balance = new ParallelRaceGroup(
                autoBalance,
                velocityMaintainer,
                new WaitCommand(6.75)
        );
        this.addCommands(balance);
        this.addCommands(brake);
    }
}
