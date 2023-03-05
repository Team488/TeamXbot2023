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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import javax.inject.Inject;
import java.util.ArrayList;
import java.util.List;

public class EjectLowThenExitLowProgram extends SequentialCommandGroup {

    @Inject
    public EjectLowThenExitLowProgram(
            PoseSubsystem pose,
            EjectCollectorCommand ejectCollectorCommand,
            CollectorSubsystem collector,
            SwerveSimpleTrajectoryCommand exitBottom,
            AutoBalanceCommand autoBalance,
            VelocityMaintainerCommand velocityMaintainer,
            BrakeCommand brake) {

        // Force set our current position - facing the grid station, ready to score our cone.
        var forcePosition = new InstantCommand(
                () -> {
                    var translation =
                            AutoLandmarks.convertBlueToRedIfNeeded(AutoLandmarks.blueScoringPositionTwo)
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

        exitBottom.setMaxPower(0.50);
        exitBottom.setKeyPointsProvider(
                () -> {
                    // Get out of the community
                    var exitCommunityPoint =
                            AutoLandmarks.convertBlueToRedIfNeeded(AutoLandmarks.blueLowerCheckpointOutsideCommunity);
                    XbotSwervePoint exitCommunitySwervePoint = new XbotSwervePoint(
                            exitCommunityPoint.getX(),
                            exitCommunityPoint.getY(),
                            pose.rotateAngleBasedOnAlliance(Rotation2d.fromDegrees(0)).getDegrees(),
                            3.0);
                    return new ArrayList<>(List.of(exitCommunitySwervePoint));
                });

        this.addCommands(exitBottom);
    }
}
