package competition.subsystems.drive.commands;

import competition.auto_programs.AutoLandmarks;
import competition.operator_interface.OperatorInterface;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.drive.control_logic.HeadingModule;
import xbot.common.subsystems.feedback.XRumbleManager;

import javax.inject.Inject;
import java.util.Arrays;
import java.util.List;

public class SwerveToNearestScoringPositionCommand extends SwerveSimpleTrajectoryCommand {

    private final DoubleProperty maxTravelDistanceProp;
    private final DoubleProperty bufferSpaceProp;
    private final XRumbleManager rumbleManager;

    private Pose2d targetPose;

    @Inject
    public SwerveToNearestScoringPositionCommand(
            DriveSubsystem drive,
            PoseSubsystem pose,
            PropertyFactory pf,
            HeadingModule.HeadingModuleFactory headingModuleFactory,
            OperatorInterface oi
    ) {
        super(drive, pose, pf, headingModuleFactory);
        this.rumbleManager = oi.driverGamepad.getRumbleManager();

        pf.setPrefix("SwerveToNearestScoringPositionCommand");
        maxTravelDistanceProp = pf.createPersistentProperty("Maximum travel distance inches", 100);
        bufferSpaceProp = pf.createPersistentProperty("Buffer space", 2);
    }

    @Override
    public final void initialize() {
        Pose2d currentPose = this.pose.getCurrentPose2d();
        Pose2d newPose = getTargetPose();

        double bufferAllianceMultiplier = (pose.getAlliance() == DriverStation.Alliance.Blue) ? -1.0 : 1.0;
        Pose2d intermediatePoint = currentPose
                .interpolate(newPose, 0.5)
                .transformBy(new Transform2d(
                        new Translation2d(bufferSpaceProp.get() * bufferAllianceMultiplier, 0),
                        new Rotation2d(0)));
        targetPose = newPose;
        log.info(String.format("Current pose is: %s; Target pose is: %s", currentPose.toString(), this.targetPose.toString()));

        double distanceToTarget = currentPose.getTranslation().getDistance(newPose.getTranslation());

        if (distanceToTarget > this.maxTravelDistanceProp.get()) {
            log.warn(String.format(
                    "Target position is %f inches from current position. This exceeds our maximum travel of %f. Not moving.",
                    distanceToTarget, this.maxTravelDistanceProp.get()));
            targetPose = currentPose;
            rumbleManager.rumbleGamepad(1.0, 0.75);

            setKeyPoints(Arrays.asList(
                    new XbotSwervePoint(targetPose, 0)
            ));
        } else {
            setKeyPoints(Arrays.asList(
                    new XbotSwervePoint(intermediatePoint, 0.5),
                    new XbotSwervePoint(targetPose, 0.5)
            ));
        }

        super.initialize();
    }

    @Override
    public final void execute() {
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        rumbleManager.stopGamepadRumble();
    }

    protected Pose2d getTargetPose() {
        return findNearestScoringPosition(pose.getCurrentPose2d(), pose.getAlliance());
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
