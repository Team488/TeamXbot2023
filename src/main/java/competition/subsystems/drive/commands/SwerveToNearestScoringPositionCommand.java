package competition.subsystems.drive.commands;

import competition.auto_programs.AutoLandmarks;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import xbot.common.math.XYPair;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.drive.control_logic.HeadingModule;

import javax.inject.Inject;
import java.util.Arrays;
import java.util.List;

public class SwerveToNearestScoringPositionCommand extends SwerveToPointCommand {

    private Pose2d targetPose;

    @Inject
    public SwerveToNearestScoringPositionCommand(
            DriveSubsystem drive,
            PoseSubsystem pose,
            PropertyFactory pf,
            HeadingModule.HeadingModuleFactory headingModuleFactory
    ) {
        super(drive, pose, pf, headingModuleFactory);
    }

    @Override
    public void initialize() {
        this.targetPose = findNearestScoringPosition(pose.getCurrentPose2d(), pose.getAlliance());
        setTargetPosition(new XYPair(targetPose.getX(), targetPose.getY()), targetPose.getRotation().getDegrees());

        super.initialize();
    }

    @Override
    public void execute() {
        super.execute();
    }

    public static List<Pose2d> getScoringPositionPoses(DriverStation.Alliance alliance) {
        return Arrays.asList(
                AutoLandmarks.convertBlueToRedIfNeeded(AutoLandmarks.blueScoringPositionOne, alliance),
                AutoLandmarks.convertBlueToRedIfNeeded(AutoLandmarks.blueScoringPositionTwo, alliance),
                AutoLandmarks.convertBlueToRedIfNeeded(AutoLandmarks.blueScoringPositionThree, alliance),
                AutoLandmarks.convertBlueToRedIfNeeded(AutoLandmarks.blueScoringPositionFour, alliance),
                AutoLandmarks.convertBlueToRedIfNeeded(AutoLandmarks.blueScoringPositionFive, alliance),
                AutoLandmarks.convertBlueToRedIfNeeded(AutoLandmarks.blueScoringPositionSix, alliance),
                AutoLandmarks.convertBlueToRedIfNeeded(AutoLandmarks.blueScoringPositionSeven, alliance),
                AutoLandmarks.convertBlueToRedIfNeeded(AutoLandmarks.blueScoringPositionEight, alliance),
                AutoLandmarks.convertBlueToRedIfNeeded(AutoLandmarks.blueScoringPositionNine, alliance)
        );
    }

    public static Pose2d findNearestScoringPosition(Pose2d currentPose, DriverStation.Alliance alliance) {
        List<Pose2d> poses = getScoringPositionPoses(alliance);
        return currentPose.nearest(poses);
    }
}
