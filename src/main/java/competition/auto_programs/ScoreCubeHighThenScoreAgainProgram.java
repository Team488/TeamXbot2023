package competition.auto_programs;

import competition.commandgroups.CollectionSequenceCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreCubeHighThenScoreAgainProgram extends SequentialCommandGroup {

    public ScoreCubeHighThenScoreAgainProgram(
            ScoreCubeHighThenLeaveProgram phaseOne,
            CollectionSequenceCommandGroup deployGroundCollector) {

        // We already have a score high then leave, but now we want to grab another game piece
        this.addCommands(phaseOne);

        // Rotate

        // Deploy collector and wait until it's actually deployed
        this.addCommands(deployGroundCollector);
    }
}
