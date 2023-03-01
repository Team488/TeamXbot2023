package competition.commandgroups;

import competition.subsystems.arm.UnifiedArmSubsystem;
import competition.subsystems.arm.commands.SetArmsToKeyArmPositionCommand;
import competition.subsystems.claw.CloseClawCommand;
import competition.subsystems.claw.OpenClawCommand;
import competition.subsystems.collector.CollectorSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import javax.inject.Inject;

public class MoveCollectedGamepieceToArmCommandGroup extends SequentialCommandGroup {

    @Inject
    public MoveCollectedGamepieceToArmCommandGroup(
            OpenClawCommand openClawCommand,
            CloseClawCommand closeClawCommand,
            CollectorSubsystem collector,
            SetArmsToKeyArmPositionCommand setArmsToCollectPositionCommand,
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

        // Eject the gamepiece
        this.addCommands(collector.getEjectThenStopCommand().withTimeout(0.1));

    }
}
