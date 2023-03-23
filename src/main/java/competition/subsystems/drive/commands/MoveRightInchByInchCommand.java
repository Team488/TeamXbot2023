package competition.subsystems.drive.commands;

import javax.inject.Inject;
import javax.inject.Provider;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import xbot.common.math.XYPair;

public class MoveRightInchByInchCommand extends SequentialCommandGroup {
    @Inject
    MoveRightInchByInchCommand(SwerveToPointCommand swerveToPointCommand, BrakeCommand brake) {
        swerveToPointCommand.setRobotRelativeMotion();
        swerveToPointCommand.setTargetPosition(new XYPair(0,3), 0);
        this.addCommands(swerveToPointCommand.withTimeout(1.0));
        this.addCommands(brake);
    }
}