package competition.commandgroups;

import competition.subsystems.arm.UnifiedArmSubsystem;
import competition.subsystems.arm.commands.SimpleXZRouterCommand;
import competition.subsystems.claw.OpenClawCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import javax.inject.Inject;
import javax.inject.Provider;

public class ScoreCubeMidCommandGroup extends SequentialCommandGroup {
    @Inject
    ScoreCubeMidCommandGroup(OpenClawCommand openClaw,
                             Provider<SimpleXZRouterCommand> setArmPosProvider,
                             UnifiedArmSubsystem arm) {
        // Set scoring mode to the relevant game piece
        this.addCommands(arm.createSetGamePieceModeCommand(UnifiedArmSubsystem.GamePieceMode.Cube));

        //move arm to the key position
        var moveArmToPosition = setArmPosProvider.get();
        moveArmToPosition.setKeyPointFromKeyArmPosition(
                UnifiedArmSubsystem.KeyArmPosition.MidGoal, UnifiedArmSubsystem.RobotFacing.Forward);

        this.addCommands(moveArmToPosition.withTimeout(5.0));

        this.addCommands(openClaw);

    }
}
