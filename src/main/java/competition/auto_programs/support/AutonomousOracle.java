package competition.auto_programs.support;

import competition.auto_programs.AutoLandmarks;
import competition.subsystems.arm.UnifiedArmSubsystem;
import competition.subsystems.drive.commands.XbotSwervePoint;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import org.apache.log4j.LogManager;
import org.apache.log4j.Logger;
import xbot.common.command.NamedInstantCommand;
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

    private MantlePrepPosition mantlePrepPosition = MantlePrepPosition.InsideCommunity;
    private final StringProperty mantlePrepPositionProp;


    // Enable/disable switches for various parts of the autonomous program.
    private final BooleanProperty enableDrivePhaseOne;
    private final BooleanProperty enableAcquireGamePiece;
    private final BooleanProperty enableMoveToScore;
    private final BooleanProperty enableSecondScore;
    private final BooleanProperty enableBalance;

    PoseSubsystem pose;

    Logger log = LogManager.getLogger(AutonomousOracle.class);
    private Field2d oracleField;

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

        oracleField = new Field2d();
        SmartDashboard.putData("OracleField", oracleField);

        setEnableAcquireGamePiece(true);
        setInitialScoringLocationIndex(6);
        setLane(Lane.Top);

        oracleField.setRobotPose(getInitialPoseInMeters());
        setTrajectoryForDisplay("Phase1", getTrajectoryForDrivePhaseOne());
        setTrajectoryForDisplay("Scoring", getTrajectoryForScoring());
        setTrajectoryForDisplay("Balance", getTrajectoryForBalance());
    }

    // -------------------------------------------
    // Basic setters/getters as needed
    // -------------------------------------------
    public void setInitialScoringLocationIndex(int index) {
        initialScoringLocationIndex.set(index);
        var initialPose = AutoLandmarks.convertBlueToRedIfNeeded(getLocationForScoringPositionIndex(index));
        initialLocation = initialPose.getTranslation();
        initialHeading = initialPose.getRotation();
    }

    public void setLane(Lane lane) {
        this.lane = lane;
        laneProp.set(lane.toString());
    }

    public void setInitialGamePiece(UnifiedArmSubsystem.GamePieceMode gamePiece) {
        this.initialGamePiece = gamePiece;
        initialGamePieceProp.set(gamePiece.toString());
    }

    public UnifiedArmSubsystem.GamePieceMode getInitialGamePiece() {
        return initialGamePiece;
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

    public ScoringMode getSecondScoringMode() {
        return secondScoringMode;
    }

    public void setMantlePrepPosition(MantlePrepPosition position) {
        this.mantlePrepPosition = position;
        mantlePrepPositionProp.set(position.toString());
    }

    public void setEnableDrivePhaseOne(boolean enable) {
        enableDrivePhaseOne.set(enable);
    }

    public boolean getEnableDrivePhaseOne() {
        return enableDrivePhaseOne.get();
    }

    public void setEnableAcquireGamePiece(boolean enable) {
        enableAcquireGamePiece.set(enable);
    }

    public boolean getEnableAcquireGamePiece() {
        return enableAcquireGamePiece.get();
    }

    public void setEnableMoveToScore(boolean enable) {
        enableMoveToScore.set(enable);
    }

    public boolean getEnableMoveToScore() {
        return enableMoveToScore.get();
    }

    public void setEnableSecondScore(boolean enable) {
        enableSecondScore.set(enable);
    }

    public boolean getEnableSecondScore() {
        return enableSecondScore.get();
    }

    public void setEnableBalance(boolean enable) {
        enableBalance.set(enable);
    }

    public boolean getEnableBalance() {
        return enableBalance.get();
    }

    public Pose2d getInitialPoseInMeters() {
        return new Pose2d(initialLocation.times(1.0 / PoseSubsystem.INCHES_IN_A_METER), initialHeading);
    }

    public ScoringMode getInitialScoringMode() {
        return initialScoringMode;
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

    public List<XbotSwervePoint> getTrajectoryForScoring() {
        ArrayList<XbotSwervePoint> points = new ArrayList<>();

        switch (lane) {
            case Top:
                // Mostly the reverse of how we got here, except now we're trying to go to a specific scoring position
                points.add(createXbotSwervePoint(AutoLandmarks.blueUpperCheckpointOutsideCommunity, Rotation2d.fromDegrees(-180), 1.0));
                points.add(createXbotSwervePoint(AutoLandmarks.blueUpperCommunitySideMidCheckpoint, Rotation2d.fromDegrees(-180), 1.0));
                points.add(createXbotSwervePoint(
                        getLocationForScoringPositionIndex((int)secondScoringLocationIndex.get()), Rotation2d.fromDegrees(-180), 1.0));
                break;
            case Bottom:
                // Mostly the reverse of how we got here, except now we're trying to go to a specific scoring position
                points.add(createXbotSwervePoint(AutoLandmarks.blueLowerCheckpointOutsideCommunity, Rotation2d.fromDegrees(-180), 1.0));
                points.add(createXbotSwervePoint(AutoLandmarks.blueLowerCommunitySideMidCheckpoint, Rotation2d.fromDegrees(-180), 1.0));
                points.add(createXbotSwervePoint(
                        getLocationForScoringPositionIndex((int)secondScoringLocationIndex.get()), Rotation2d.fromDegrees(-180), 1.0));
                break;
            default: // default to middle
            case Middle:
                // Mostly the reverse of how we got here, except now we're trying to go to a specific scoring position
                points.add(createXbotSwervePoint(AutoLandmarks.blueToUpperAndLowerFieldCheckpoint, Rotation2d.fromDegrees(-180), 1.0));
                points.add(createXbotSwervePoint(AutoLandmarks.blueToUpperAndLowerCommunityCheckpoint, Rotation2d.fromDegrees(-180), 1.0));
                points.add(createXbotSwervePoint(
                        getLocationForScoringPositionIndex((int)secondScoringLocationIndex.get()), Rotation2d.fromDegrees(-180), 1.0));
                break;
        }

        return points;
    }

    public List<XbotSwervePoint> getTrajectoryForBalance() {
            ArrayList<XbotSwervePoint> points = new ArrayList<>();

            switch (mantlePrepPosition) {
                default: // Default to InsideCommunity
                case InsideCommunity: // we're facing outwards here, so 0 degrees
                    points.add(createXbotSwervePoint(AutoLandmarks.blueToUpperAndLowerCommunityCheckpoint, Rotation2d.fromDegrees(0), 1.0));
                    points.add(createXbotSwervePoint(AutoLandmarks.blueChargeStationCenter, Rotation2d.fromDegrees(0), 1.0));
                    break;
                case OutsideCommunity: // we're facing inwards here, so -180 degrees
                    points.add(createXbotSwervePoint(AutoLandmarks.blueToUpperAndLowerFieldCheckpoint, Rotation2d.fromDegrees(-180), 1.0));
                    points.add(createXbotSwervePoint(AutoLandmarks.blueChargeStationCenter, Rotation2d.fromDegrees(-180), 1.0));
                    break;
            }



            return points;
    }

    public List<XbotSwervePoint> getTrajectoryToBalanceChargePlate() {
        ArrayList<XbotSwervePoint> points = new ArrayList<>();

        return points;
    }


    // -------------------------------------------
    // Helper methods
    // -------------------------------------------

    /**
     * Handles blue/red alliance conversion
     * @param targetLocation X/Y coordinates on the field
     * @param targetHeading Desired heading of the robot
     * @param durationInSeconds How long the robot should interpolate along this segment
     * @return A new XbotSwervePoint (handling blue/red alliance location) with the given parameters
     */
    private XbotSwervePoint createXbotSwervePoint(Pose2d targetLocation, Rotation2d targetHeading, double durationInSeconds) {
        var translation = AutoLandmarks.convertBlueToRedIfNeeded(targetLocation);
        var heading = pose.rotateAngleBasedOnAlliance(targetHeading);
        return new XbotSwervePoint(translation.getX(), translation.getY(), heading.getDegrees(), durationInSeconds);
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

    // -------------------------------------------
    // Factory methods for setting all the interesting properties of the autonomous mode
    // -------------------------------------------

    public NamedInstantCommand createInitialScoringPositionCommand(int scoringPositionIndex) {
        return new NamedInstantCommand("Set Initial Scoring Position" + scoringPositionIndex, () -> setInitialScoringLocationIndex(scoringPositionIndex));
    }

    public NamedInstantCommand createSetLaneCommand(Lane lane) {
        return new NamedInstantCommand("Set Lane" + lane.toString(), () -> setLane(lane));
    }

    public NamedInstantCommand createSetInitialGamePieceModeCommand(UnifiedArmSubsystem.GamePieceMode gamePieceMode) {
        return new NamedInstantCommand("Set Initial Game Piece Mode" + gamePieceMode.toString(), () -> setInitialGamePiece(gamePieceMode));
    }

    public NamedInstantCommand createInitialScoringModeCommand(ScoringMode scoringMode) {
        return new NamedInstantCommand("Set Initial Scoring Mode" + scoringMode.toString(), () -> setInitialScoringMode(scoringMode));
    }

    public NamedInstantCommand createSecondScoringPositionCommand(int scoringPositionIndex) {
        return new NamedInstantCommand("Set Second Scoring Position" + scoringPositionIndex, () -> setSecondScoringLocationIndex(scoringPositionIndex));
    }

    public NamedInstantCommand createSecondScoringModeCommand(ScoringMode scoringMode) {
        return new NamedInstantCommand("Set Second Scoring Mode" + scoringMode.toString(), () -> setSecondScoringMode(scoringMode));
    }

    public NamedInstantCommand createSecondGamePieceModeCommand(UnifiedArmSubsystem.GamePieceMode gamePieceMode) {
        return new NamedInstantCommand("Set Second Game Piece Mode" + gamePieceMode.toString(), () -> setSecondGamePiece(gamePieceMode));
    }

    public NamedInstantCommand createMantlePrepPositionCommand(MantlePrepPosition mantlePrepPosition) {
        return new NamedInstantCommand("Set Mantle Prep Position" + mantlePrepPosition.toString(), () -> setMantlePrepPosition(mantlePrepPosition));
    }

    // -------------------------------------------
    // Autonomous "plans", mostly meant for testing
    // -------------------------------------------

    public WrapperCommand createTopLaneOmniAuto() {
        return new NamedInstantCommand("Top Lane Omni Auto", () -> {
            setLane(Lane.Top);
            setInitialScoringLocationIndex(7);
            setInitialScoringMode(ScoringMode.High);
            setInitialGamePiece(UnifiedArmSubsystem.GamePieceMode.Cone);

            setSecondGamePiece(UnifiedArmSubsystem.GamePieceMode.Cube);
            setSecondScoringMode(ScoringMode.High);
            setSecondScoringLocationIndex(8);

            setMantlePrepPosition(MantlePrepPosition.InsideCommunity);

            setEnableDrivePhaseOne(true);
            setEnableAcquireGamePiece(true);
            setEnableMoveToScore(true);
            setEnableSecondScore(true);
            setEnableBalance(true);
        }).ignoringDisable(true);
    }

    // -------------------------------------------
    // Trajectory display methods
    // -------------------------------------------

    public void setTrajectoryForDisplay(String trajectoryName, Trajectory trajectory) {
        oracleField.getObject(trajectoryName).setTrajectory(trajectory);
    }

    public void setTrajectoryForDisplay(String trajectoryName, List<XbotSwervePoint> swervePoints) {
        setTrajectoryForDisplay(trajectoryName, XbotSwervePoint.generateTrajectory(swervePoints));
    }
}
