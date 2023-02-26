
package competition.commandgroups;

import competition.subsystems.arm.UnifiedArmSubsystem;
import competition.subsystems.arm.commands.SimpleSafeArmRouterCommand;
import competition.subsystems.claw.CloseClawCommand;
import competition.subsystems.claw.OpenClawCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import javax.inject.Inject;
import javax.inject.Provider;
import java.util.function.Supplier;

/**
 * Scores a game piece. Leaves the arm in scoring position with the claw open.
 */
public class ScoreGamepieceCommandGroup extends SequentialCommandGroup {

    // Set up default suppliers for scoring. Use the defaults we expect to use in most of
    // our auto programs.
    Supplier<UnifiedArmSubsystem.GamePieceMode> gamePieceModeSupplier =
            () -> UnifiedArmSubsystem.GamePieceMode.Cube;
    Supplier<UnifiedArmSubsystem.KeyArmPosition> armPositionSupplier =
            () -> UnifiedArmSubsystem.KeyArmPosition.HighGoal;
    Supplier<UnifiedArmSubsystem.RobotFacing> robotFacingSupplier =
            () -> UnifiedArmSubsystem.RobotFacing.Backward;

    @Inject
    ScoreGamepieceCommandGroup(OpenClawCommand openClaw,
                               CloseClawCommand closeClaw,
                               Provider<SimpleSafeArmRouterCommand> setArmPosProvider,
                               UnifiedArmSubsystem arm) {

        // Set scoring mode to the relevant game piece
        this.addCommands(arm.createSetGamePieceModeCommand(gamePieceModeSupplier.get()));

        //move arm to the key position
        var moveArmToPosition = setArmPosProvider.get();
        moveArmToPosition.setTarget(armPositionSupplier.get(), robotFacingSupplier.get());

        this.addCommands(moveArmToPosition);

        var openClawAndWait = new ParallelDeadlineGroup(new WaitCommand(0.5), openClaw);
    }

    public void setGamePieceModeSupplier(Supplier<UnifiedArmSubsystem.GamePieceMode> gamePieceModeSupplier) {
        this.gamePieceModeSupplier = gamePieceModeSupplier;
    }

    public void setArmPositionSupplier(Supplier<UnifiedArmSubsystem.KeyArmPosition> armPositionSupplier) {
        this.armPositionSupplier = armPositionSupplier;
    }

    public void setRobotFacingSupplier(Supplier<UnifiedArmSubsystem.RobotFacing> robotFacingSupplier) {
        this.robotFacingSupplier = robotFacingSupplier;
    }
}
