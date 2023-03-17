package competition.commandgroups;

import competition.subsystems.arm.UnifiedArmSubsystem;
import competition.subsystems.arm.commands.SimpleXZRouterCommand;
import competition.subsystems.claw.ClawGripperMotorSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import javax.inject.Inject;
import javax.inject.Provider;

public class ScoreCubeMidCommandGroup extends SequentialCommandGroup {
    @Inject
    ScoreCubeMidCommandGroup(ClawGripperMotorSubsystem claw,
                             Provider<SimpleXZRouterCommand> setArmPosProvider,
                             UnifiedArmSubsystem arm) {
        // Set scoring mode to the relevant game piece
        this.addCommands(arm.createSetGamePieceModeCommand(UnifiedArmSubsystem.GamePieceMode.Cube));

        //move arm to the key position
        var moveArmToPosition = setArmPosProvider.get();
        moveArmToPosition.setKeyPointFromKeyArmPosition(
                UnifiedArmSubsystem.KeyArmPosition.MidGoal, UnifiedArmSubsystem.RobotFacing.Forward);

        this.addCommands(moveArmToPosition.withTimeout(5.0));

        var ejectGamePiece = claw.setEject(-1);
        var ejectGamePieceAndWait = new ParallelDeadlineGroup(new WaitCommand(1), ejectGamePiece);
        this.addCommands(ejectGamePieceAndWait);

        var stopClawCommand = new InstantCommand(() -> claw.setStopped());
        this.addCommands(stopClawCommand);
    }
}
