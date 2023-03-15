package competition.subsystems.drive.commands;

import competition.operator_interface.OperatorInterface;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import xbot.common.math.MathUtils;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.drive.control_logic.HeadingModule;

import javax.inject.Inject;
import java.util.List;

public class SwerveToNextScoringPositionCommand extends SwerveToNearestScoringPositionCommand {

    private TargetDirection direction;

    @Inject
    public SwerveToNextScoringPositionCommand(
            DriveSubsystem drive,
            PoseSubsystem pose,
            PropertyFactory pf,
            HeadingModule.HeadingModuleFactory headingModuleFactory,
            OperatorInterface oi
    ) {
        super(drive, pose, pf, headingModuleFactory, oi);
    }

    /**
     * Sets the target direction of the movement (relative to the driver station)
     * @param direction The direction to translate
     */
    public void setDirection(TargetDirection direction) {
        this.direction = direction;
    }

    @Override
    protected Pose2d getTargetPose() {
        return findNextScoringPosition(pose.getCurrentPose2d(), pose.getAlliance(), this.direction);
    }

    public static Pose2d findNextScoringPosition(Pose2d currentPose, DriverStation.Alliance alliance, TargetDirection direction) {
        Pose2d nearestPosition = findNearestScoringPosition(currentPose, alliance);
        List<Pose2d> allScoringPositions = getScoringPositionPoses(alliance);

        int currentIndex = allScoringPositions.indexOf(nearestPosition);

        int directionMultiplier = (direction == TargetDirection.Left) ? 1 : -1;
        int allianceMultiplier = (alliance == DriverStation.Alliance.Blue) ? 1 : -1;

        int nextIndex = currentIndex + (1 * directionMultiplier * allianceMultiplier);
        nextIndex = MathUtils.constrainInt(nextIndex, 0, allScoringPositions.size() - 1);
        return allScoringPositions.get(nextIndex);
    }

    public enum TargetDirection {
        Left,
        Right
    }
}
