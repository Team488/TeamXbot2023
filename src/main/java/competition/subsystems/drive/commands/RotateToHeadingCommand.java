package competition.subsystems.drive.commands;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import xbot.common.subsystems.drive.control_logic.HeadingModule;

import javax.inject.Inject;

public class RotateToHeadingCommand extends TurnLeft180DegreesCommand{

    @Inject
    public RotateToHeadingCommand(DriveSubsystem drive, PoseSubsystem pose, HeadingModule.HeadingModuleFactory headingModuleFactory) {
        super(drive, pose, headingModuleFactory);
    }

    public void setGoalHeading(double heading) {
        goalHeading = Rotation2d.fromDegrees(heading);
    }
}
