package competition.commandgroups;

import competition.subsystems.arm.UnifiedArmSubsystem;
import competition.subsystems.arm.commands.SetArmsToKeyArmPositionCommand;
import competition.subsystems.claw.ClawGripperMotorSubsystem;
import competition.subsystems.claw.CloseClawCommand;
import competition.subsystems.claw.OpenClawCommand;
import competition.subsystems.collector.CollectorSubsystem;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import javax.inject.Inject;
import javax.inject.Provider;

public class MoveCollectedGamepieceToArmCommandGroup extends SequentialCommandGroup {

    @Inject
    public MoveCollectedGamepieceToArmCommandGroup(
            OpenClawCommand openClawCommand,
            CloseClawCommand closeClawCommand,
            CollectorSubsystem collector,
            Provider<SetArmsToKeyArmPositionCommand> setArmsToKeyArmPositionCommandProvider,
            ClawGripperMotorSubsystem clawMotors,
            UnifiedArmSubsystem arms) {

        // Open claw and wait
        this.addCommands(openClawCommand.withTimeout(0.5));

        // Move arm to the "collect" position
        var setArmsToCollectPositionCommand = setArmsToKeyArmPositionCommandProvider.get();
        setArmsToCollectPositionCommand.setTargetSupplier(
                () -> UnifiedArmSubsystem.KeyArmPosition.AcquireFromCollector,
                () -> UnifiedArmSubsystem.RobotFacing.Forward);
        this.addCommands(setArmsToCollectPositionCommand);

        // Close claw and wait
        this.addCommands(closeClawCommand.withTimeout(0.5));

        // If we are in cube mode, we can use the simple "eject and pull away" combo.
        // If we are in cone mode, we instead need to:
        // - Collector eject while claw intakes for 0.25 seconds.
        // - then move the arm to the "prepare" position.

        // First, the cube scenario
        // Eject the gamepiece, and move the arm out a bit.
        var cubePullOutGamePieceCommand = setArmsToKeyArmPositionCommandProvider.get();
        cubePullOutGamePieceCommand.setTargetSupplier(
                () -> UnifiedArmSubsystem.KeyArmPosition.PrepareToAcquireFromCollector,
                () -> UnifiedArmSubsystem.RobotFacing.Forward);

        var waitThenPullArm = new WaitCommand(0.1).andThen(cubePullOutGamePieceCommand);
        var cubePullSequence = collector.getEjectThenStopCommand()
                .withTimeout(0.25)
                .alongWith(waitThenPullArm);

        // Now, the cone scenario
        var conePullOutGamePieceCommand = setArmsToKeyArmPositionCommandProvider.get();
        conePullOutGamePieceCommand.setTargetSupplier(
                () -> UnifiedArmSubsystem.KeyArmPosition.PrepareToAcquireFromCollector,
                () -> UnifiedArmSubsystem.RobotFacing.Forward);

        double coneEjectTimeInSeconds = 0.25;
        var clawConeIntake = clawMotors.createIntakeCommand().withTimeout(coneEjectTimeInSeconds);
        var collectorConeEject = collector.getEjectThenStopCommand()
                .withTimeout(coneEjectTimeInSeconds);

        var conePullSequence = (collectorConeEject.alongWith(clawConeIntake)).alongWith(conePullOutGamePieceCommand);

        var coneOrCubePullSequence = new ConditionalCommand(cubePullSequence, conePullSequence, () -> arms.isCubeMode());

        this.addCommands(coneOrCubePullSequence);

    }
}
