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

public class ScoreConeHighCommandGroup extends SequentialCommandGroup {
    @Inject
    ScoreConeHighCommandGroup(ClawGripperMotorSubsystem claw, Provider<SimpleXZRouterCommand> setArmPosProvider, UnifiedArmSubsystem arm){
        this.addCommands(arm.createSetGamePieceModeCommand(UnifiedArmSubsystem.GamePieceMode.Cone));

        var moveArmToHigh = setArmPosProvider.get();
        moveArmToHigh.setKeyPointFromKeyArmPosition(UnifiedArmSubsystem.KeyArmPosition.HighGoal, UnifiedArmSubsystem.RobotFacing.Forward);

        this.addCommands(moveArmToHigh.withTimeout(5));
        var ejectGamePiece = claw.setEject(-1);
        var ejectGamePieceAndWait = new ParallelDeadlineGroup(new WaitCommand(1),ejectGamePiece);
        this.addCommands(ejectGamePieceAndWait);

        var stopClawCommand = new InstantCommand(()-> claw.setStopped());
        this.addCommands(stopClawCommand);

    }
}
