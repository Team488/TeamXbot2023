package competition.commandgroups;

import competition.subsystems.arm.UnifiedArmSubsystem;
import competition.subsystems.arm.commands.SetArmsToKeyArmPositionCommand;
import competition.subsystems.claw.CloseClawCommand;
import competition.subsystems.claw.OpenClawCommand;
import competition.subsystems.collector.CollectorSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import javax.inject.Inject;

public class MoveCollectedGamepieceToArmCommandGroup extends SequentialCommandGroup {

    @Inject
    public MoveCollectedGamepieceToArmCommandGroup(
            OpenClawCommand openClawCommand,
            CloseClawCommand closeClawCommand,
            CollectorSubsystem collector,
            SetArmsToKeyArmPositionCommand setArmsToCollectPositionCommand,
            SetArmsToKeyArmPositionCommand pullOutGamePieceCommand,
            UnifiedArmSubsystem arms) {

        // Open claw and wait
        this.addCommands(openClawCommand.withTimeout(0.5));

        // Move arm to the "collect" position
        setArmsToCollectPositionCommand.setTargetSupplier(
                () -> UnifiedArmSubsystem.KeyArmPosition.AcquireFromCollector,
                () -> UnifiedArmSubsystem.RobotFacing.Forward);
        this.addCommands(setArmsToCollectPositionCommand);

        // Close claw and wait
        this.addCommands(closeClawCommand.withTimeout(0.5));

        // Eject the gamepiece, and move the arm out a bit.
        pullOutGamePieceCommand.setTargetSupplier(
                () -> UnifiedArmSubsystem.KeyArmPosition.PrepareToAcquireFromCollector,
                () -> UnifiedArmSubsystem.RobotFacing.Forward);

        var waitThenPullArm = new WaitCommand(0.1).andThen(pullOutGamePieceCommand);
        var pullSequence = collector.getEjectThenStopCommand()
                .withTimeout(0.25)
                .alongWith(waitThenPullArm);

        this.addCommands(pullSequence);

    }
}
