
package competition.commandgroups;

import competition.subsystems.arm.UnifiedArmSubsystem;
import competition.subsystems.arm.commands.SimpleXZRouterCommand;
import competition.subsystems.claw.ClawGripperMotorSubsystem;
import competition.subsystems.claw.OpenClawCommand;
import edu.wpi.first.wpilibj2.command.*;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;
import javax.inject.Provider;
import java.util.function.Supplier;

/**
 * Scores a game piece. Leaves the arm in scoring position with the claw open.
 */
public class ScoreCubeHighCommandGroup extends SequentialCommandGroup {

    @Inject
    ScoreCubeHighCommandGroup(ClawGripperMotorSubsystem claw,
                              Provider<SimpleXZRouterCommand> setArmPosProvider,
                              UnifiedArmSubsystem arm) {
        // Set scoring mode to the relevant game piece
        this.addCommands(arm.createSetGamePieceModeCommand(UnifiedArmSubsystem.GamePieceMode.Cube));

        //move arm to the key position
        var moveArmToPosition = setArmPosProvider.get();
        moveArmToPosition.setKeyPointFromKeyArmPosition(
                UnifiedArmSubsystem.KeyArmPosition.HighGoal, UnifiedArmSubsystem.RobotFacing.Forward);

        this.addCommands(moveArmToPosition.withTimeout(5.0));

        var ejectGamePiece = claw.setEject(-1);
        var ejectGamePieceAndWait = new ParallelDeadlineGroup(new WaitCommand(1.0), (Command) ejectGamePiece);
        this.addCommands(ejectGamePieceAndWait);

        var stopClawCommand = new InstantCommand(() -> claw.setStopped());

        this.addCommands(stopClawCommand);
    }
}
