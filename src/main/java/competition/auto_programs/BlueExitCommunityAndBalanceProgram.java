package competition.auto_programs;

import competition.subsystems.arm.UnifiedArmSubsystem;
import competition.subsystems.drive.commands.AutoBalanceCommand;
import competition.subsystems.drive.commands.SwerveSimpleTrajectoryCommand;
import competition.subsystems.drive.commands.XbotSwervePoint;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

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
                                       SwerveSimpleTrajectoryCommand swerveSimpleTrajectoryCommand,
                                       AutoLandmarks landmarks,
                                       UnifiedArmSubsystem armSubsystem){
        InstantCommand setPitchCompensation = new InstantCommand(() -> armSubsystem.setPitchCompensation(true));
        this.addCommands(setPitchCompensation);
        // Swerve points are X, Y, degrees, and seconds spent going to that point.
        XbotSwervePoint turnAroundTowardsGoal = new XbotSwervePoint
                (landmarks.blueLowerGamePieceSideMidCheckpoint.getX(), landmarks.blueLowerGamePieceSideMidCheckpoint.getY(), -180, 0.5);
        XbotSwervePoint moveTowardsFieldSideCheckpoint = new XbotSwervePoint
                (landmarks.blueToUpperAndLowerFieldCheckpoint.getX(), landmarks.blueToUpperAndLowerFieldCheckpoint.getY(), -180, 0.5);
        XbotSwervePoint goTowardsChargeStation = new XbotSwervePoint
                (landmarks.blueChargeStationCenter.getX(), landmarks.blueChargeStationCenter.getY(), -180, 1.0);

        swerveSimpleTrajectoryCommand.setKeyPoints(
                new ArrayList<>(List.of(
                        turnAroundTowardsGoal,
                        moveTowardsFieldSideCheckpoint,
                        goTowardsChargeStation
                )));
        var autoBalance = autoBalanceCommandProvider.get();

        this.addCommands(swerveSimpleTrajectoryCommand);
        this.addCommands(autoBalance);

        InstantCommand resetPitchCompensation = new InstantCommand(() -> armSubsystem.setPitchCompensation(false));
        this.addCommands(resetPitchCompensation);


    }
}
