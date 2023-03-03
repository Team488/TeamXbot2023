package competition.auto_programs;

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
import javax.inject.Provider;
import java.util.ArrayList;
import java.util.List;

/**
 * Drives straight at the charge pad and attempts to balance
 */
public class BlueScoringPositionFiveToBalanceProgram extends SequentialCommandGroup {
    @Inject
    BlueScoringPositionFiveToBalanceProgram(Provider<AutoBalanceCommand> autoBalanceCommandProvider,
                                            SwerveSimpleTrajectoryCommand mantleChargePlate,
                                            PoseSubsystem pose,
                                            VelocityMaintainerCommand velocityMaintainer, BrakeCommand brake){
        // Force set our current position - facing the charge station, with some distance to get some speed.
        var forcePosition = new InstantCommand(
                () -> {
                    var translation =
                            AutoLandmarks.convertBlueToRedIfNeeded(AutoLandmarks.blueScoringPositionFive)
                                    .getTranslation().times(1.0 / PoseSubsystem.INCHES_IN_A_METER);
                    var rotation = pose.rotateAngleBasedOnAlliance(Rotation2d.fromDegrees(0));
                    pose.setCurrentPoseInMeters(new Pose2d(translation, rotation));
                }
        );
        this.addCommands(forcePosition);

        // Drive forward and pray that we mantle the charge plate
        mantleChargePlate.setKeyPointsProvider(
                () -> {
                    var chargeStationMantlePoint =
                            AutoLandmarks.convertBlueToRedIfNeeded(AutoLandmarks.blueChargeStationCenter);
                    XbotSwervePoint goTowardsChargeStation = new XbotSwervePoint(
                            chargeStationMantlePoint.getX(),
                            chargeStationMantlePoint.getY(),
                            pose.rotateAngleBasedOnAlliance(Rotation2d.fromDegrees(0)).getDegrees(),
                            1.0);
                    return new ArrayList<>(List.of(goTowardsChargeStation));
                }
        );

        var mantleWithTimeout = mantleChargePlate.withTimeout(3);
        // TODO: Change this to 1.0 (or something close to it) for competition
        mantleChargePlate.setMaxPower(0.5);
        this.addCommands(mantleWithTimeout);


        // Combine all the commands into one huge sequential command, then deadline that against ~14.5 seconds
        // That way, in the last 0.5 second we can engage the brake.
        var balance = new ParallelRaceGroup(
                autoBalanceCommandProvider.get(),
                velocityMaintainer,
                new WaitCommand(11.5)
        );
        this.addCommands(balance);
        this.addCommands(brake);
    }
}
