package competition.auto_programs;

import competition.subsystems.drive.commands.SwerveToPointCommand;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import xbot.common.math.XYPair;

import javax.inject.Inject;
import javax.inject.Provider;

public class BasicMobilityPoints extends SequentialCommandGroup{
    @Inject
    BasicMobilityPoints(Provider<SwerveToPointCommand> swerveToPointCommandProvider, PoseSubsystem pose){
        var moveForward = swerveToPointCommandProvider.get();
        moveForward.setRobotRelativeMotion();
        moveForward.setMaxPower(0.5);
        moveForward.setMaxTurningPower(0.5);
        moveForward.setTargetPosition(new XYPair(40,0), 0);
        this.addCommands(moveForward);
    }
}
