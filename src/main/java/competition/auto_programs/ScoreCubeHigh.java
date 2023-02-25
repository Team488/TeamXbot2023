
package competition.auto_programs;

import competition.subsystems.arm.UnifiedArmSubsystem;
import competition.subsystems.arm.commands.SetArmsToPositionCommand;
import competition.subsystems.claw.CloseClawCommand;
import competition.subsystems.claw.OpenClawCommand;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.drive.commands.SwerveToPointCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import xbot.common.math.XYPair;

import javax.inject.Inject;
import javax.inject.Provider;

public class ScoreCubeHigh extends SequentialCommandGroup {
    @Inject
    ScoreCubeHigh(Provider<OpenClawCommand> openClawProvider,
                  Provider<CloseClawCommand> closeClawProvider,
                  Provider<SetArmsToPositionCommand> setArmPosProvider,
                  Provider<SwerveToPointCommand> swerveToPointProvider,
                  DriveSubsystem drive) {
        //move arm to high
        var moveArmToHigh = setArmPosProvider.get();
        moveArmToHigh.setTargetPosition(UnifiedArmSubsystem.KeyArmPosition.HighGoal, UnifiedArmSubsystem.RobotFacing.Backward);

        this.addCommands(moveArmToHigh);

        //open claw and close claw
        var openClaw = openClawProvider.get();
        var closeClaw = closeClawProvider.get();

        var openClawThenClose = new ParallelRaceGroup(
                new ParallelCommandGroup(openClaw, new WaitCommand(1), closeClaw),
                new WaitCommand(5));
        this.addCommands(openClawThenClose);

        //retract arm
        var retractArm = setArmPosProvider.get();
        retractArm.setTargetPosition(UnifiedArmSubsystem.KeyArmPosition.FullyRetracted, UnifiedArmSubsystem.RobotFacing.Backward);
        this.addCommands(retractArm);

        //drive out of community zone
        var moveOutOfCommunity = swerveToPointProvider.get();
        moveOutOfCommunity.setFieldRelativeMotion();
        moveOutOfCommunity.setMaxPower(0.5);
        moveOutOfCommunity.setTargetPosition(new XYPair(185, 176), 0);

        var driveOut = new ParallelRaceGroup(
                moveOutOfCommunity,
                new WaitCommand(3));
        this.addCommands(driveOut);
        //stop robot
        drive.stop();
    }
}
