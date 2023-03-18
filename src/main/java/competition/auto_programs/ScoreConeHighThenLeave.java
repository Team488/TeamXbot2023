package competition.auto_programs;

import competition.subsystems.arm.commands.SimpleXZRouterCommand;
import competition.subsystems.claw.CloseClawCommand;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.drive.commands.SwerveToPointCommand;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import javax.inject.Inject;

public class ScoreConeHighThenLeave extends SequentialCommandGroup {
    @Inject
    ScoreConeHighThenLeave(SimpleXZRouterCommand retractArm,
                           SwerveToPointCommand moveOutOfCommunity,
                           DriveSubsystem drive,
                           PoseSubsystem pose){
        var forcePosition = new InstantCommand(() -> {
            var translation = AutoLandmarks.convertBlueToRedIfNeeded(AutoLandmarks.blueScoringPositionNine).getTranslation().times(1/PoseSubsystem.INCHES_IN_A_METER);
        })
                //add rumble to drivers controller before continueing

    }
}
