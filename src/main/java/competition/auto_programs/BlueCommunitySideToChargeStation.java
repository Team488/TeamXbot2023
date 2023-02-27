package competition.auto_programs;

import competition.subsystems.drive.commands.AutoBalanceCommand;
import competition.subsystems.drive.commands.BrakeCommand;
import competition.subsystems.drive.commands.SwerveSimpleTrajectoryCommand;
import competition.subsystems.drive.commands.VelocityMaintainerCommand;
import competition.subsystems.drive.commands.XbotSwervePoint;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import javax.inject.Inject;
import javax.inject.Provider;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/**
 * Drives straight at the charge pad and attempts to balance
 */
public class BlueCommunitySideToChargeStation extends SequentialCommandGroup {
    @Inject
    BlueCommunitySideToChargeStation(Provider<AutoBalanceCommand> autoBalanceCommandProvider,
                                     SwerveSimpleTrajectoryCommand mantleChargePlate,
                                     AutoLandmarks landmarks, PoseSubsystem pose,
                                     VelocityMaintainerCommand velocityMaintainer, BrakeCommand brake){
        // Force set our current position - facing the charge station, with some distance to get some speed.
        var forcePosition = new InstantCommand(
                () -> {
                    var translation =
                            AutoLandmarks.convertBlueToRedIfNeeded(AutoLandmarks.blueScoringPositionFive)
                                    .getTranslation();
                    var rotation = pose.rotateAngleBasedOnAlliance(Rotation2d.fromDegrees(0));
                }
        );
        this.addCommands(forcePosition);

        // Drive forward and pray that we mantle the charge plate
        mantleChargePlate.setKeyPointsProvider(
                () -> {
                    var chargeStationMantlePoint =
                            AutoLandmarks.convertBlueToRedIfNeeded(landmarks.blueChargeStationMantleFromLeft);
                    XbotSwervePoint goTowardsChargeStation = new XbotSwervePoint
                            (chargeStationMantlePoint.getX(), chargeStationMantlePoint.getY(), 0, 1.0);
                    return new ArrayList<>(List.of(goTowardsChargeStation));
                }
        );

        // TODO: Change this to 1.0 (or something close to it) for competition
        mantleChargePlate.setMaxPower(0.5);

        // Combine all the commands into one huge sequential command, then deadline that against ~14.5 seconds
        // That way, in the last 0.5 second we can engage the brake.
        var mantleAndBalance = new SequentialCommandGroup(
                mantleChargePlate.deadlineWith(new WaitCommand(2.0)), // Mantle but give up after 2 sec.
                new ParallelCommandGroup(
                        autoBalanceCommandProvider.get(),
                        velocityMaintainer
                )
        ).deadlineWith(new WaitCommand(14.5));

        this.addCommands(mantleAndBalance);
        this.addCommands(brake);
    }
}
