package competition.auto_programs;

import competition.subsystems.collector.CollectorSubsystem;
import competition.subsystems.collector.commands.EjectCollectorCommand;
import competition.subsystems.drive.commands.AutoBalanceCommand;
import competition.subsystems.drive.commands.BrakeCommand;
import competition.subsystems.drive.commands.SwerveSimpleTrajectoryCommand;
import competition.subsystems.drive.commands.VelocityMaintainerCommand;
import competition.subsystems.drive.commands.XbotSwervePoint;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import javax.inject.Inject;
import java.util.ArrayList;
import java.util.List;

public class EjectLowThenBalanceWithMobilityProgram extends SequentialCommandGroup {

    @Inject
    public EjectLowThenBalanceWithMobilityProgram(
            PoseSubsystem pose,
            EjectCollectorCommand ejectCollectorCommand,
            SwerveSimpleTrajectoryCommand mantleChargePlate,
            CollectorSubsystem collector,
            AutoBalanceCommand autoBalance,
            VelocityMaintainerCommand velocityMaintainer,
            BrakeCommand brake
    ) {
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

        // Eject the cone for 1 second.
        var ejectAndWait = ejectCollectorCommand.withTimeout(1);
        this.addCommands(ejectAndWait);
        this.addCommands(new InstantCommand(() -> collector.stop()));

        mantleChargePlate.setMaxPower(0.75);
        mantleChargePlate.setKeyPointsProvider(
                () -> {
                    // back up 6 inches over 0.5 seconds
                    var backUpPointPreMirrored = AutoLandmarks.blueScoringPositionFive;
                    backUpPointPreMirrored.getTranslation().plus(new Translation2d(6, 0));
                    var backUpPoint = AutoLandmarks.convertBlueToRedIfNeeded(backUpPointPreMirrored);
                    XbotSwervePoint backUpPointSwerve = new XbotSwervePoint(
                            backUpPoint.getX(),
                            backUpPoint.getY(),
                            pose.rotateAngleBasedOnAlliance(Rotation2d.fromDegrees(-180)).getDegrees(),
                            0.5);

                    // back up an additional 3 inches while rotating over 1 second
                    var backUpPoint2PreMirrored = AutoLandmarks.blueScoringPositionFive;
                    backUpPoint2PreMirrored.getTranslation().plus(new Translation2d(9, 0));
                    var backUpPoint2 = AutoLandmarks.convertBlueToRedIfNeeded(backUpPoint2PreMirrored);
                    XbotSwervePoint backUpPoint2Swerve = new XbotSwervePoint(
                            backUpPoint2.getX(),
                            backUpPoint2.getY(),
                            pose.rotateAngleBasedOnAlliance(Rotation2d.fromDegrees(0)).getDegrees(),
                            1.0);

                    // Go straight over the charge plate to get those mobility points over 2 seconds
                    var mobilityPoint =
                            AutoLandmarks.convertBlueToRedIfNeeded(AutoLandmarks.blueToUpperAndLowerFieldCheckpoint);
                    XbotSwervePoint mobilitySwervePoint = new XbotSwervePoint(
                            mobilityPoint.getX(),
                            mobilityPoint.getY(),
                            pose.rotateAngleBasedOnAlliance(Rotation2d.fromDegrees(0)).getDegrees(),
                            2.0);


                    // Get on that charge station over 1 second
                    var chargeStationMantlePoint =
                            AutoLandmarks.convertBlueToRedIfNeeded(AutoLandmarks.blueChargeStationCenter);
                    XbotSwervePoint getOnChargeStation = new XbotSwervePoint(
                            chargeStationMantlePoint.getX(),
                            chargeStationMantlePoint.getY(),
                            pose.rotateAngleBasedOnAlliance(Rotation2d.fromDegrees(0)).getDegrees(),
                            1.0);
                    return new ArrayList<>(List.of(backUpPointSwerve, backUpPoint2Swerve, mobilitySwervePoint, getOnChargeStation));
                });
        this.addCommands(mantleChargePlate);

        // With a 15-second budget, and subtracting 1 second for scoring, and 4.5 seconds for mantle, we have 9.5 seconds to balance.
        // to make 100% sure we engage the brakes, set a timeout for just under that.
        var balance = new ParallelRaceGroup(
                autoBalance,
                velocityMaintainer,
                new WaitCommand(9.25)
        );
        this.addCommands(balance);
        this.addCommands(brake);
    }
}
