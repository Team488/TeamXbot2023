
package competition.commandgroups;

import competition.subsystems.arm.UnifiedArmSubsystem;
import competition.subsystems.arm.commands.SimpleXZRouterCommand;
import competition.subsystems.claw.CloseClawCommand;
import competition.subsystems.claw.OpenClawCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import javax.inject.Inject;
import javax.inject.Provider;
import java.util.function.Supplier;

/**
 * Scores a game piece. Leaves the arm in scoring position with the claw open.
 */
public class ScoreCubeHighCommandGroup extends SequentialCommandGroup {

    @Inject
    ScoreCubeHighCommandGroup(OpenClawCommand openClaw,
                              Provider<SimpleXZRouterCommand> setArmPosProvider,
                              UnifiedArmSubsystem arm) {

        // Set scoring mode to the relevant game piece
        this.addCommands(arm.createSetGamePieceModeCommand(UnifiedArmSubsystem.GamePieceMode.Cube));

        //move arm to the key position
        var moveArmToPosition = setArmPosProvider.get();
        moveArmToPosition.setKeyPointFromKeyArmPosition(
                UnifiedArmSubsystem.KeyArmPosition.HighGoal, UnifiedArmSubsystem.RobotFacing.Forward);

        this.addCommands(moveArmToPosition.withTimeout(5.0));

        var openClawAndWait = new ParallelDeadlineGroup(new WaitCommand(0.5), openClaw);

        this.addCommands(openClawAndWait);
    }
}
