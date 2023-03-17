package competition.auto_programs;

import competition.commandgroups.CollectionSequenceCommandGroup;
import competition.commandgroups.ScoreCubeHighCommandGroup;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.drive.commands.SwerveToPointCommand;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import javax.inject.Inject;

public class ScoreCubeHighThenScoreCubeMidProgram extends SequentialCommandGroup {
    @Inject
    ScoreCubeHighThenScoreCubeMidProgram(ScoreCubeHighThenLeaveProgram scoreCubeHigh,
                                         CollectionSequenceCommandGroup collect,
                                         SwerveToPointCommand moveOutCommunity,
                                         DriveSubsystem drive,
                                         PoseSubsystem pose){
        this.addCommands(scoreCubeHigh);
        //rotate and collect




    }
}
