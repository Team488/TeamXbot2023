package competition.commandgroups;

import competition.subsystems.arm.UnifiedArmSubsystem;
import competition.subsystems.arm.commands.SimpleXZRouterCommand;
import competition.subsystems.claw.ClawGripperMotorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import javax.inject.Inject;
import javax.inject.Provider;

public class ScoreGamepieceCommandGroupFactory {

    ClawGripperMotorSubsystem clawMotors;
    Provider<SimpleXZRouterCommand> setArmPosProvider;
    UnifiedArmSubsystem arm;

    @Inject
    ScoreGamepieceCommandGroupFactory(ClawGripperMotorSubsystem clawMotors,
                                      Provider<SimpleXZRouterCommand> setArmPosProvider,
                                      UnifiedArmSubsystem arm) {
        this.clawMotors = clawMotors;
        this.setArmPosProvider = setArmPosProvider;
        this.arm = arm;
    }

    public SequentialCommandGroup create(UnifiedArmSubsystem.KeyArmPosition lowMedHigh, boolean retractAfterwards) {
        var group = new SequentialCommandGroup();

        //move arm to the key position
        var moveArmToPosition = setArmPosProvider.get();
        moveArmToPosition.setKeyPointFromKeyArmPositionProvider(
                () -> lowMedHigh,
                () -> UnifiedArmSubsystem.RobotFacing.Forward);
        group.addCommands(moveArmToPosition);

        // Eject the game piece onto the grid
        var ejectGamePiece = clawMotors.setEject(-0.75);
        var ejectGamePieceAndWait = new ParallelDeadlineGroup(new WaitCommand(1.0), ejectGamePiece);
        group.addCommands(ejectGamePieceAndWait);

        // Stop the claw motors
        var stopClawCommand = new InstantCommand(() -> clawMotors.setStopped());
        group.addCommands(stopClawCommand);

        // Optionally retract the arm
        var retractArm = setArmPosProvider.get();
        retractArm.setKeyPointFromKeyArmPositionProvider(
                () -> UnifiedArmSubsystem.KeyArmPosition.PrepareToAcquireFromCollector,
                () -> UnifiedArmSubsystem.RobotFacing.Forward);

        var optionallyRetractArm = new ConditionalCommand(
                retractArm,
                new InstantCommand(),
                () -> retractAfterwards);
        group.addCommands(optionallyRetractArm);

        return group;
    }
}