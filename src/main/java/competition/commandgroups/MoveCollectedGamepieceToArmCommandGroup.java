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
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

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
            UnifiedArmSubsystem arms,
            PropertyFactory pf) {

        pf.setPrefix(this.getClass().getName());
        double coneClawIntakeTime = 0.35;
        double coneCollectorEjectTime = 0.35;
        double cubeCollectorEjectTime = 0.25;
        double cubeClawIntakeTime = 0.2; //we can change this number if needed

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

        // If we are in cube mode, use the same system as cone mode, but shorter intake
        // If we are in cone mode, we instead need to:
        // - Collector eject while claw intakes for 0.25 seconds.
        // - then move the arm to the "prepare" position.

        // First, the cube scenario
        // Eject the gamepiece, and move the arm out a bit.
        var cubePullOutGamePieceCommand = setArmsToKeyArmPositionCommandProvider.get();
        cubePullOutGamePieceCommand.setTargetSupplier(
                () -> UnifiedArmSubsystem.KeyArmPosition.PrepareToAcquireFromCollector,
                () -> UnifiedArmSubsystem.RobotFacing.Forward);

        var clawCubeIntake = clawMotors.createIntakeCommand()
                .withTimeout(cubeClawIntakeTime)
                .andThen(clawMotors.createStopCommand().withTimeout(0.01));
        
        var collectorCubeEject = collector.getEjectThenStopCommand()
                .withTimeout(cubeCollectorEjectTime);        

        var cubePullSequence = (collectorCubeEject.alongWith(clawCubeIntake)).andThen(cubePullOutGamePieceCommand);

        // Now, the cone scenario
        var conePullOutGamePieceCommand = setArmsToKeyArmPositionCommandProvider.get();
        conePullOutGamePieceCommand.setTargetSupplier(
                () -> UnifiedArmSubsystem.KeyArmPosition.PrepareToAcquireFromCollector,
                () -> UnifiedArmSubsystem.RobotFacing.Forward);

        var clawConeIntake = clawMotors.createIntakeCommand()
                .withTimeout(coneClawIntakeTime)
                .andThen(clawMotors.createStopCommand().withTimeout(0.01));

        var collectorConeEject = collector.getEjectThenStopCommand()
                .withTimeout(coneCollectorEjectTime);

        var conePullSequence = (collectorConeEject.alongWith(clawConeIntake)).andThen(conePullOutGamePieceCommand);

        var coneOrCubePullSequence = new ConditionalCommand(cubePullSequence, conePullSequence, () -> arms.isCubeMode());

        this.addCommands(coneOrCubePullSequence);

    }
}
