package competition.auto_programs;

import competition.subsystems.arm.UnifiedArmSubsystem;
import competition.subsystems.drive.commands.AutoBalanceCommand;
import competition.subsystems.drive.commands.BrakeCommand;
import competition.subsystems.drive.commands.SwerveSimpleTrajectoryCommand;
import competition.subsystems.drive.commands.VelocityMaintainerCommand;
import competition.subsystems.drive.commands.XbotSwervePoint;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import javax.inject.Inject;
import javax.inject.Provider;
import java.util.ArrayList;
import java.util.List;

/**
 * Drives out of the community (to get Park points) then drives to the charge pad and attempts to balance
 */
public class BlueExitCommunityAndBalanceProgram extends SequentialCommandGroup {
    @Inject
    BlueExitCommunityAndBalanceProgram(Provider<AutoBalanceCommand> autoBalanceCommandProvider,
                                       SwerveSimpleTrajectoryCommand mantleChargePlate,
                                       AutoLandmarks landmarks, VelocityMaintainerCommand velocityMaintainer,
                                       PoseSubsystem pose, BrakeCommand brake) {

        // Force set our current position - just about to exit the community
        var forcePosition = new InstantCommand(
                () -> {
                    var translation =
                            AutoLandmarks.convertBlueToRedIfNeeded(AutoLandmarks.blueBelowChargeStation)
                                    .getTranslation();
                    var rotation = pose.rotateAngleBasedOnAlliance(Rotation2d.fromDegrees(-180));
                }
        );
        this.addCommands(forcePosition);

        // TODO: Increase power at competition.
        mantleChargePlate.setMaxPower(0.5);
        mantleChargePlate.setKeyPointsProvider(
                () -> {
                    var blueLowerGamePieceSideMidCheckpoint =
                            AutoLandmarks.convertBlueToRedIfNeeded(landmarks.blueLowerGamePieceSideMidCheckpoint);
                    var blueToUpperAndLowerFieldCheckpoint =
                            AutoLandmarks.convertBlueToRedIfNeeded(landmarks.blueToUpperAndLowerFieldCheckpoint);
                    var blueChargeStationMantleFromRight =
                            AutoLandmarks.convertBlueToRedIfNeeded(landmarks.blueChargeStationMantleFromRight);

                    XbotSwervePoint blueLowerGamePieceSideMidCheckpointPoint = new XbotSwervePoint
                            (blueLowerGamePieceSideMidCheckpoint.getX(),
                                    blueLowerGamePieceSideMidCheckpoint.getY(),
                                    -180,
                                    1.0);
                    XbotSwervePoint blueToUpperAndLowerFieldCheckpointPoint = new XbotSwervePoint
                            (blueToUpperAndLowerFieldCheckpoint.getX(),
                                    blueToUpperAndLowerFieldCheckpoint.getY(),
                                    -180,
                                    1.0);
                    XbotSwervePoint blueChargeStationMantleFromRightPoint = new XbotSwervePoint
                            (blueChargeStationMantleFromRight.getX(),
                                    blueChargeStationMantleFromRight.getY(),
                                    -180,
                                    0.5);

                    return new ArrayList<>(List.of(blueLowerGamePieceSideMidCheckpointPoint,
                            blueToUpperAndLowerFieldCheckpointPoint,
                            blueChargeStationMantleFromRightPoint));
                }
        );

        var mantleAndBalance = new SequentialCommandGroup(
                mantleChargePlate.deadlineWith(new WaitCommand(5.0)), // Mantle but give up after 5 sec.
                new ParallelCommandGroup(
                        autoBalanceCommandProvider.get(),
                        velocityMaintainer
                )
        ).deadlineWith(new WaitCommand(14.5));

        this.addCommands(mantleAndBalance);
        this.addCommands(brake);

    }
}
