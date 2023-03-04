package competition.auto_programs.support;

import competition.auto_programs.AutoLandmarks;
import competition.subsystems.arm.UnifiedArmSubsystem;
import competition.subsystems.drive.commands.XbotSwervePoint;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.apache.log4j.LogManager;
import org.apache.log4j.Logger;
import xbot.common.properties.BooleanProperty;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;
import xbot.common.properties.StringProperty;

import javax.inject.Inject;
import javax.inject.Singleton;
import java.util.ArrayList;
import java.util.List;

@Singleton
public class AutonomousOracle {

    public enum Lane {
        Top,
        Middle,
        Bottom
    }

    public enum ScoringMode {
        Eject,
        Low,
        Medium,
        High
    }

    public enum MantlePrepPosition {
        InsideCommunity,
        OutsideCommunity
    }

    // Values used to compute actions in the autonomous program
    private Translation2d initialLocation = AutoLandmarks.blueScoringPositionFive.getTranslation();
    private Rotation2d initialHeading = Rotation2d.fromDegrees(-180);
    private final DoubleProperty initialScoringLocationIndex;

    private Lane lane = Lane.Middle;
    private final StringProperty laneProp;

    private UnifiedArmSubsystem.GamePieceMode initialGamePiece = UnifiedArmSubsystem.GamePieceMode.Cone;
    private final StringProperty initialGamePieceProp;

    private ScoringMode initialScoringMode = ScoringMode.Eject;
    private final StringProperty initialScoringModeProp;

    private Pose2d secondScoringPosition = AutoLandmarks.blueScoringPositionSix;
    private final DoubleProperty secondScoringLocationIndex;

    private UnifiedArmSubsystem.GamePieceMode secondGamePiece = UnifiedArmSubsystem.GamePieceMode.Cone;
    private final StringProperty secondGamePieceProp;

    private ScoringMode secondScoringMode = ScoringMode.Eject;
    private final StringProperty secondScoringModeProp;

    private MantlePrepPosition mantlePrepPosition;
    private final StringProperty mantlePrepPositionProp;


    // Enable/disable switches for various parts of the autonomous program.
    private final BooleanProperty enableDrivePhaseOne;
    private final BooleanProperty enableAcquireGamePiece;
    private final BooleanProperty enableMoveToScore;
    private final BooleanProperty enableSecondScore;
    private final BooleanProperty enableBalance;

    PoseSubsystem pose;

    Logger log = LogManager.getLogger(AutonomousOracle.class);

    /**
     * Holds onto a large set of autonomous settings to power a very generic autonomous program. Primarily a bunch of
     * setters and getters, with a little bit of route composition logic.
     */
    @Inject
    public AutonomousOracle(PropertyFactory pf, PoseSubsystem pose) {
        this.pose = pose;
        pf.setPrefix("AutonomousOracle");
        initialScoringLocationIndex = pf.createEphemeralProperty("InitialScoringLocationIndex", 5);
        laneProp = pf.createEphemeralProperty("Lane", "Middle");
        initialGamePieceProp = pf.createEphemeralProperty("InitialGamePiece", "Cone");
        initialScoringModeProp = pf.createEphemeralProperty("InitialScoringMode", "Eject");

        secondScoringLocationIndex = pf.createEphemeralProperty("SecondScoringLocationIndex", 6);
        secondGamePieceProp = pf.createEphemeralProperty("SecondGamePiece", "Cone");
        secondScoringModeProp = pf.createEphemeralProperty("SecondScoringMode", "Eject");
        mantlePrepPositionProp = pf.createEphemeralProperty("MantlePrepPosition", "OutsideCommunity");

        enableDrivePhaseOne = pf.createEphemeralProperty("EnableDrivePhaseOne", false);
        enableAcquireGamePiece = pf.createEphemeralProperty("EnableAcquireGamePiece", false);
        enableMoveToScore = pf.createEphemeralProperty("EnableMoveToScore", false);
        enableSecondScore = pf.createEphemeralProperty("EnableSecondScore", false);
        enableBalance = pf.createEphemeralProperty("EnableBalance", true);
    }

    // -------------------------------------------
    // Basic setters/getters as needed
    // -------------------------------------------
    public void setInitialScoringLocationIndex(int index) {
        initialScoringLocationIndex.set(index);
        initialLocation = getLocationForScoringPositionIndex(index).getTranslation();
    }

    public void setLane(Lane lane) {
        this.lane = lane;
        laneProp.set(lane.toString());
    }

    public void setInitialGamePiece(UnifiedArmSubsystem.GamePieceMode gamePiece) {
        this.initialGamePiece = gamePiece;
        initialGamePieceProp.set(gamePiece.toString());
    }

    public void setInitialScoringMode(ScoringMode scoringMode) {
        this.initialScoringMode = scoringMode;
        initialScoringModeProp.set(scoringMode.toString());
    }

    public void setSecondScoringLocationIndex(int index) {
        secondScoringLocationIndex.set(index);
        secondScoringPosition = getLocationForScoringPositionIndex(index);
    }

    public void setSecondGamePiece(UnifiedArmSubsystem.GamePieceMode gamePiece) {
        this.secondGamePiece = gamePiece;
        secondGamePieceProp.set(gamePiece.toString());
    }

    public void setSecondScoringMode(ScoringMode scoringMode) {
        this.secondScoringMode = scoringMode;
        secondScoringModeProp.set(scoringMode.toString());
    }

    public void setMantlePrepPosition(MantlePrepPosition position) {
        this.mantlePrepPosition = position;
        mantlePrepPositionProp.set(position.toString());
    }

    public void setEnableDrivePhaseOne(boolean enable) {
        enableDrivePhaseOne.set(enable);
    }

    public void setEnableAcquireGamePiece(boolean enable) {
        enableAcquireGamePiece.set(enable);
    }

    public void setEnableMoveToScore(boolean enable) {
        enableMoveToScore.set(enable);
    }

    public void setEnableSecondScore(boolean enable) {
        enableSecondScore.set(enable);
    }

    public void setEnableBalance(boolean enable) {
        enableBalance.set(enable);
    }


    // -------------------------------------------
    // Route composition
    // -------------------------------------------

    public List<XbotSwervePoint> getTrajectoryForDrivePhaseOne() {
        // Right now two major possibilities:
        // We are going to pick up a new game piece, so we should go to the natural point
        // Otherwise, we are trying to get simple mobility points before we balance

        if (enableAcquireGamePiece.get()) {
            return getTrajectoryForDrivePhaseOneWithGamePiece();
        } else {
            return getTrajectoryForDrivePhaseOneMobility();
        }
    }

    private List<XbotSwervePoint> getTrajectoryForDrivePhaseOneWithGamePiece() {

        ArrayList<XbotSwervePoint> points = new ArrayList<>();

        switch (lane) {
            case Top:
                points.add(createXbotSwervePoint(AutoLandmarks.blueUpperCommunitySideMidCheckpoint, Rotation2d.fromDegrees(0), 1.0));
                points.add(createXbotSwervePoint(AutoLandmarks.blueGamePieceUpper, Rotation2d.fromDegrees(0), 1.0));
                break;
            case Bottom:
                points.add(createXbotSwervePoint(AutoLandmarks.blueLowerCommunitySideMidCheckpoint, Rotation2d.fromDegrees(0), 1.0));
                points.add(createXbotSwervePoint(AutoLandmarks.blueGamePieceLower, Rotation2d.fromDegrees(0), 1.0));
                break;
            default: // default to middle
            case Middle:
                points.add(createXbotSwervePoint(AutoLandmarks.blueToUpperAndLowerCommunityCheckpoint, Rotation2d.fromDegrees(0), 0.5));
                points.add(createXbotSwervePoint(AutoLandmarks.blueToUpperAndLowerFieldCheckpoint, Rotation2d.fromDegrees(0), 2.0));
                points.add(createXbotSwervePoint(AutoLandmarks.blueGamePieceSecondUpper, Rotation2d.fromDegrees(0), 1.0));
                break;
        }

        return points;
    }

    private List<XbotSwervePoint> getTrajectoryForDrivePhaseOneMobility() {
        ArrayList<XbotSwervePoint> points = new ArrayList<>();

        switch (lane) {
            case Top:
                points.add(createXbotSwervePoint(AutoLandmarks.blueUpperCommunitySideMidCheckpoint, Rotation2d.fromDegrees(0), 1.0));
                // Turn around, prepare to mantle
                points.add(createXbotSwervePoint(AutoLandmarks.blueUpperCheckpointOutsideCommunity, Rotation2d.fromDegrees(-180), 1.0));
                break;
            case Bottom:
                points.add(createXbotSwervePoint(AutoLandmarks.blueLowerCommunitySideMidCheckpoint, Rotation2d.fromDegrees(0), 1.0));
                // Turn around, prepare to mantle
                points.add(createXbotSwervePoint(AutoLandmarks.blueLowerCheckpointOutsideCommunity, Rotation2d.fromDegrees(-180), 1.0));
                break;
            default: // default to middle
            case Middle:
            points.add(createXbotSwervePoint(AutoLandmarks.blueToUpperAndLowerCommunityCheckpoint, Rotation2d.fromDegrees(0), 0.5));
            points.add(createXbotSwervePoint(AutoLandmarks.blueToUpperAndLowerFieldCheckpoint, Rotation2d.fromDegrees(0), 2.0));
            // Turn around, prepare to mantle
            points.add(createXbotSwervePoint(AutoLandmarks.blueToUpperAndLowerFieldCheckpoint, Rotation2d.fromDegrees(-180), 1.0));
            break;
        }
        return points;
    }


    // -------------------------------------------
    // Helper methods
    // -------------------------------------------

    private XbotSwervePoint createXbotSwervePoint(Pose2d targetLocation, Rotation2d targetHeading, double durationInSeconds) {
        var translation = AutoLandmarks.convertBlueToRedIfNeeded(targetLocation);
        var heading = pose.rotateAngleBasedOnAlliance(targetHeading);
        return new XbotSwervePoint(translation.getX(), translation.getY(), targetHeading.getDegrees(), durationInSeconds);
    }

    private Pose2d getLocationForScoringPositionIndex(int index) {
        switch (index) {
            case 1:
                return AutoLandmarks.blueScoringPositionOne;
            case 2:
                return AutoLandmarks.blueScoringPositionTwo;
            case 3:
                return AutoLandmarks.blueScoringPositionThree;
            case 4:
                return AutoLandmarks.blueScoringPositionFour;
            case 5:
                return AutoLandmarks.blueScoringPositionFive;
            case 6:
                return AutoLandmarks.blueScoringPositionSix;
            case 7:
                return AutoLandmarks.blueScoringPositionSeven;
            case 8:
                return AutoLandmarks.blueScoringPositionEight;
            case 9:
                return AutoLandmarks.blueScoringPositionNine;
            default:
                // When in doubt, the middle.
                log.info("Somebody attempted to set an invalid scoring location index: " + index + ". Using 5 instead");
                return AutoLandmarks.blueScoringPositionFive;
        }
    }
}
