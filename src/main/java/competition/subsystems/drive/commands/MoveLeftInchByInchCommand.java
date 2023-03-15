package competition.subsystems.drive.commands;

import javax.inject.Inject;
import javax.inject.Provider;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import xbot.common.math.XYPair;

public class MoveLeftInchByInchCommand extends SequentialCommandGroup {
    @Inject
    MoveLeftInchByInchCommand(Provider<SwerveToPointCommand> swerveToPointCommandProvider){
        var moveLeft = swerveToPointCommandProvider.get();
        moveLeft.setRobotRelativeMotion();
        moveLeft.setTargetPosition(new XYPair(0,-2), 0);
        this.addCommands(moveLeft);
    }
}