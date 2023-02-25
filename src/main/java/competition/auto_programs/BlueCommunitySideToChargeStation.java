package competition.auto_programs;

import competition.subsystems.arm.UnifiedArmSubsystem;
import competition.subsystems.drive.commands.AutoBalanceCommand;
import competition.subsystems.drive.commands.SwerveSimpleTrajectoryCommand;
import competition.subsystems.drive.commands.XbotSwervePoint;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import javax.inject.Provider;
import java.util.ArrayList;
import java.util.List;

public class BlueCommunitySideToChargeStation extends SequentialCommandGroup {
    BlueCommunitySideToChargeStation(Provider<AutoBalanceCommand> autoBalanceCommandProvider, SwerveSimpleTrajectoryCommand swerveSimpleTrajectoryCommand, AutoLandmarks landmarks, UnifiedArmSubsystem armSubsystem){
        InstantCommand setPitchCompensation = new InstantCommand(() -> armSubsystem.setPitchCompensation(true));
        this.addCommands(setPitchCompensation);
        // Swerve points are X, Y, degrees, and seconds spent going to that point.
        XbotSwervePoint backAwayFromGoal = new XbotSwervePoint(landmarks.blueToUpperAndLowerCommunityCheckpoint.getX(), landmarks.blueToUpperAndLowerCommunityCheckpoint.getY(), -180, 0.5);
        XbotSwervePoint turnAroundTowardsChargeStation = new XbotSwervePoint(landmarks.blueToUpperAndLowerCommunityCheckpoint.getX(), landmarks.blueToUpperAndLowerCommunityCheckpoint.getY(), 0, 0.5);
        XbotSwervePoint goTowardsChargeStation = new XbotSwervePoint(landmarks.blueChargeStation.getX(), landmarks.blueChargeStation.getY(), 0, 1.0);

        swerveSimpleTrajectoryCommand.setKeyPoints(
                new ArrayList<>(List.of(
                        backAwayFromGoal,
                        turnAroundTowardsChargeStation,
                        goTowardsChargeStation
                )));
        var autoBalance = autoBalanceCommandProvider.get();

        this.addCommands(swerveSimpleTrajectoryCommand);
        this.addCommands(autoBalance);

        InstantCommand resetPitchCompensation = new InstantCommand(() -> armSubsystem.setPitchCompensation(false));
        this.addCommands(resetPitchCompensation);


    }
}
