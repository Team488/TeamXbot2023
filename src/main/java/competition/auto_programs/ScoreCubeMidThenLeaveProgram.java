package competition.auto_programs;

import competition.commandgroups.ScoreCubeHighCommandGroup;
import competition.commandgroups.ScoreCubeMidCommandGroup;
import competition.subsystems.arm.commands.SimpleXZRouterCommand;
import competition.subsystems.claw.CloseClawCommand;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.drive.commands.SwerveToPointCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import javax.inject.Inject;

public class ScoreCubeMidThenLeaveProgram extends SequentialCommandGroup {
    @Inject
    ScoreCubeMidThenLeaveProgram(ScoreCubeMidCommandGroup scoreCubeMidCommandGroup,
                             SimpleXZRouterCommand retractArm,
                             CloseClawCommand closeClaw,
                             SwerveToPointCommand moveOutOfCommunity,
                             DriveSubsystem drive) {

        this.addCommands(scoreCubeMidCommandGroup);

        moveOutOfCommunity.setFieldRelativeMotion();
        moveOutOfCommunity.setMaxPower(0.5);

        //convert our target if needed from blue to red
        moveOutOfCommunity.setTargetSupplier(
                () -> {
                    var wpiXY = AutoLandmarks.convertBlueToRedIfNeeded(
                            AutoLandmarks.blueNorthOfChargingStationOutsideCommunity
                    ).getTranslation();
                    return new XYPair(wpiXY.getX(), wpiXY.getY());
                }
        );


    }
}
