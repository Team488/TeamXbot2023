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
    private Translation2d initialLocation = AutoLandmarks.blueScoringPositionFour.getTranslation();
    private Rotation2d initialHeading = Rotation2d.fromDegrees(-180);
    private final DoubleProperty initialScoringLocationIndexProp;
    private int initialScoringLocationIndex;

    private Lane lane = Lane.Middle;
    private final StringProperty laneProp;

    private UnifiedArmSubsystem.GamePieceMode initialGamePiece = UnifiedArmSubsystem.GamePieceMode.Cone;
    private final StringProperty initialGamePieceProp;

    private ScoringMode initialScoringMode = ScoringMode.High;
    private final StringProperty initialScoringModeProp;

    private Pose2d secondScoringPosition = AutoLandmarks.blueScoringPositionSix;
    private final DoubleProperty secondScoringLocationIndexProp;
    private int secondScoringLocationIndex;

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
        initialScoringLocationIndexProp = pf.createEphemeralProperty("InitialScoringLocationIndex", 4);
        laneProp = pf.createEphemeralProperty("Lane", "Middle");
        initialGamePieceProp = pf.createEphemeralProperty("InitialGamePiece", "Cone");
        initialScoringModeProp = pf.createEphemeralProperty("InitialScoringMode", "High");

        secondScoringLocationIndexProp = pf.createEphemeralProperty("SecondScoringLocationIndex", 6);
        secondGamePieceProp = pf.createEphemeralProperty("SecondGamePiece", "Cone");
        secondScoringModeProp = pf.createEphemeralProperty("SecondScoringMode", "Eject");
        mantlePrepPositionProp = pf.createEphemeralProperty("MantlePrepPosition", "InsideCommunity");

        enableDrivePhaseOne = pf.createEphemeralProperty("EnableDrivePhaseOne", false);
        enableAcquireGamePiece = pf.createEphemeralProperty("EnableAcquireGamePiece", false);
        enableMoveToScore = pf.createEphemeralProperty("EnableMoveToScore", false);
        enableSecondScore = pf.createEphemeralProperty("EnableSecondScore", false);
        enableBalance = pf.createEphemeralProperty("EnableBalance", false);

        oracleField = new Field2d();
        SmartDashboard.putData("OracleField", oracleField);

        setEnableAcquireGamePiece(false);
        setInitialScoringLocationIndex(4);
        setLane(Lane.Middle);

        oracleField.setRobotPose(getInitialPoseInMeters());
        setTrajectoryForDisplay("Phase1", getTrajectoryForDrivePhaseOne());
        setTrajectoryForDisplay("Scoring", getTrajectoryForScoring());
        setTrajectoryForDisplay("Balance", getTrajectoryForBalance());
    }

    // -------------------------------------------
    // Basic setters/getters as needed
    // -------------------------------------------
    public void setInitialScoringLocationIndex(int index) {
        initialScoringLocationIndex = index;
        initialScoringLocationIndexProp.set(index);
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
        secondScoringLocationIndex = index;
        secondScoringLocationIndexProp.set(index);
    }

    public void setSecondGamePiece(UnifiedArmSubsystem.GamePieceMode gamePiece) {
        this.secondGamePiece = gamePiece;
        secondGamePieceProp.set(gamePiece.toString());
    }

    public UnifiedArmSubsystem.GamePieceMode getSecondGamePiece() {
        return secondGamePiece;
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
        var initialPose = AutoLandmarks.convertBlueToRedIfNeeded(getLocationForScoringPositionIndex(initialScoringLocationIndex));
        initialLocation = initialPose.getTranslation();
        initialHeading = initialPose.getRotation();

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
                points.add(createXbotSwervePoint(AutoLandmarks.blueUpperCommunitySideMidCheckpoint, Rotation2d.fromDegrees(-180), 1.0));
                points.add(createXbotSwervePoint(
                        createAdjustedLandmark(AutoLandmarks.blueGamePieceUpper, 12, 0),
                        Rotation2d.fromDegrees(0), 1.0));
                break;
            case Bottom:
                points.add(createXbotSwervePoint(AutoLandmarks.blueLowerCommunitySideMidCheckpoint, Rotation2d.fromDegrees(-180), 1.0));
                points.add(createXbotSwervePoint(
                        createAdjustedLandmark(AutoLandmarks.blueGamePieceLower, 12, 0),
                        Rotation2d.fromDegrees(0), 1.0));
                break;
            default: // default to middle
            case Middle:
                points.add(createXbotSwervePoint(AutoLandmarks.blueToUpperAndLowerCommunityCheckpoint, Rotation2d.fromDegrees(0), 0.5));
                points.add(createXbotSwervePoint(AutoLandmarks.blueToUpperAndLowerFieldCheckpoint, Rotation2d.fromDegrees(0), 2.0));
                points.add(createXbotSwervePoint(
                        createAdjustedLandmark(AutoLandmarks.blueGamePieceSecondUpper, 12, 0),
                        Rotation2d.fromDegrees(0), 1.0));
                break;
        }

        return points;
    }

    private List<XbotSwervePoint> getTrajectoryForDrivePhaseOneMobility() {
        ArrayList<XbotSwervePoint> points = new ArrayList<>();

        switch (lane) {
            case Top:
                points.add(createXbotSwervePoint(AutoLandmarks.blueUpperCommunitySideMidCheckpoint, Rotation2d.fromDegrees(-180), 1.0));
                // Turn around, prepare to mantle
                points.add(createXbotSwervePoint(AutoLandmarks.blueUpperCheckpointOutsideCommunity, Rotation2d.fromDegrees(-180), 1.0));
                break;
            case Bottom:
                points.add(createXbotSwervePoint(AutoLandmarks.blueLowerCommunitySideMidCheckpoint, Rotation2d.fromDegrees(-180), 1.0));
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

        // Take the right route back.
        switch (lane) {
            case Top:
                // Mostly the reverse of how we got here, except now we're trying to go to a specific scoring position
                points.add(createXbotSwervePoint(AutoLandmarks.blueUpperCheckpointOutsideCommunity, Rotation2d.fromDegrees(-180), 1.0));
                points.add(createXbotSwervePoint(AutoLandmarks.blueUpperCommunitySideMidCheckpoint, Rotation2d.fromDegrees(-180), 1.0));
                break;
            case Bottom:
                // Mostly the reverse of how we got here, except now we're trying to go to a specific scoring position
                points.add(createXbotSwervePoint(AutoLandmarks.blueLowerCheckpointOutsideCommunity, Rotation2d.fromDegrees(-180), 1.0));
                points.add(createXbotSwervePoint(AutoLandmarks.blueLowerCommunitySideMidCheckpoint, Rotation2d.fromDegrees(-180), 1.0));
                break;
            default: // default to middle
            case Middle:
                // Mostly the reverse of how we got here, except now we're trying to go to a specific scoring position
                points.add(createXbotSwervePoint(AutoLandmarks.blueToUpperAndLowerFieldCheckpoint, Rotation2d.fromDegrees(-180), 1.0));
                points.add(createXbotSwervePoint(AutoLandmarks.blueToUpperAndLowerCommunityCheckpoint, Rotation2d.fromDegrees(-180), 1.0));
                break;
        }

        // Go to the actual scoring location.
        // First, set a point 2 feet back from the goal to visit first, to help us line up.
        points.add(createXbotSwervePoint(
                createLandmarkJustBeforeScoringPosition(
                        secondScoringLocationIndex,
                        2 * 12), Rotation2d.fromDegrees(-180), 1.0));

        // Next, go to the scoring location. For cubes we can afford to be a few inches back,
        // which may help us avoid some "grinding" against the scoring grid due to odometry issues.
        double xAdjustmentInInches = 0;
        if (secondGamePiece == UnifiedArmSubsystem.GamePieceMode.Cube) {
            xAdjustmentInInches = 3;
        }
        var scoringLandmark = createAdjustedLandmark(getLocationForScoringPositionIndex(
                secondScoringLocationIndex), xAdjustmentInInches, 0);
        points.add(createXbotSwervePoint(scoringLandmark, Rotation2d.fromDegrees(-180), 1.0));

        return points;
    }

    public List<XbotSwervePoint> getTrajectoryForPrepareToBalance() {
        ArrayList<XbotSwervePoint> points = new ArrayList<>();

        switch (mantlePrepPosition) {
            default: // Default to InsideCommunity
            case InsideCommunity: // we're facing outwards here, so 0 degrees
                points.add(createXbotSwervePoint(AutoLandmarks.blueToUpperAndLowerCommunityCheckpoint, Rotation2d.fromDegrees(0), 1.0));
                break;
            case OutsideCommunity: // we're facing inwards here, so -180 degrees
                points.add(createXbotSwervePoint(AutoLandmarks.blueToUpperAndLowerFieldCheckpoint, Rotation2d.fromDegrees(-180), 1.0));
                break;
        }
        return points;
    }

    public List<XbotSwervePoint> getTrajectoryForActualBalance() {
        ArrayList<XbotSwervePoint> points = new ArrayList<>();

        switch (mantlePrepPosition) {
            default: // Default to InsideCommunity
            case InsideCommunity: // we're facing outwards here, so 0 degrees
                points.add(createXbotSwervePoint(AutoLandmarks.blueChargeStationCenter, Rotation2d.fromDegrees(0), 0.75));
                break;
            case OutsideCommunity: // we're facing inwards here, so -180 degrees
                points.add(createXbotSwervePoint(AutoLandmarks.blueChargeStationCenter, Rotation2d.fromDegrees(-180), 0.75));
                break;
        }
        return points;
    }

    // -------------------------------------------
    // Helper methods
    // -------------------------------------------

    /**
     * Handles blue/red alliance conversion
     *
     * @param targetLocation    X/Y coordinates on the field
     * @param targetHeading     Desired heading of the robot
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

    public WrapperCommand createInitialScoringPositionCommand(int scoringPositionIndex) {
        return new NamedInstantCommand("Set Initial Scoring Position" + scoringPositionIndex,
                () -> setInitialScoringLocationIndex(scoringPositionIndex)).ignoringDisable(true);
    }

    public WrapperCommand createSetLaneCommand(Lane lane) {
        return new NamedInstantCommand("Set Lane" + lane.toString(), () -> setLane(lane)).ignoringDisable(true);
    }

    public WrapperCommand createSetInitialGamePieceModeCommand(UnifiedArmSubsystem.GamePieceMode gamePieceMode) {
        return new NamedInstantCommand("Set Initial Game Piece Mode" + gamePieceMode.toString(),
                () -> setInitialGamePiece(gamePieceMode)).ignoringDisable(true);
    }

    public WrapperCommand createInitialScoringModeCommand(ScoringMode scoringMode) {
        return new NamedInstantCommand("Set Initial Scoring Mode" + scoringMode.toString(),
                () -> setInitialScoringMode(scoringMode)).ignoringDisable(true);
    }

    public WrapperCommand createSecondScoringPositionCommand(int scoringPositionIndex) {
        return new NamedInstantCommand("Set Second Scoring Position" + scoringPositionIndex,
                () -> setSecondScoringLocationIndex(scoringPositionIndex)).ignoringDisable(true);
    }

    public WrapperCommand createSecondScoringModeCommand(ScoringMode scoringMode) {
        return new NamedInstantCommand("Set Second Scoring Mode" + scoringMode.toString(),
                () -> setSecondScoringMode(scoringMode)).ignoringDisable(true);
    }

    public WrapperCommand createSecondGamePieceModeCommand(UnifiedArmSubsystem.GamePieceMode gamePieceMode) {
        return new NamedInstantCommand("Set Second Game Piece Mode" + gamePieceMode.toString(),
                () -> setSecondGamePiece(gamePieceMode)).ignoringDisable(true);
    }

    public WrapperCommand createMantlePrepPositionCommand(MantlePrepPosition mantlePrepPosition) {
        return new NamedInstantCommand("Set Mantle Prep Position" + mantlePrepPosition.toString(),
                () -> setMantlePrepPosition(mantlePrepPosition)).ignoringDisable(true);
    }

    // -------------------------------------------
    // Autonomous "plans", some for testing, others as "favorites" for competition
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

    /**
     * For now, this is score cone high from middle then balance
     *
     * @return command to configure the autonomous mode
     */
    public WrapperCommand createFavoriteAutoOne() {
        return new NamedInstantCommand("OracleFavoriteOne", () -> {
            setLane(Lane.Middle);
            setInitialScoringLocationIndex(4);
            setInitialScoringMode(ScoringMode.High);
            setInitialGamePiece(UnifiedArmSubsystem.GamePieceMode.Cone);

            //setSecondGamePiece(UnifiedArmSubsystem.GamePieceMode.Cube);
            //setSecondScoringMode(ScoringMode.High);
            //setSecondScoringLocationIndex(8);

            setMantlePrepPosition(MantlePrepPosition.InsideCommunity);

            setEnableDrivePhaseOne(false);
            setEnableAcquireGamePiece(false);
            setEnableMoveToScore(false);
            setEnableSecondScore(false);
            setEnableBalance(true);
        }).ignoringDisable(true);
    }

    /**
     * Top lane, score, then collect (but don't attempt to score) a cube
     *
     * @return command to configure the autonomous mode
     */
    public WrapperCommand createFavoriteAutoTwo() {
        return new NamedInstantCommand("OracleFavoriteTwo", () -> {
            setLane(Lane.Top);
            setInitialScoringLocationIndex(9);
            setInitialScoringMode(ScoringMode.High);
            setInitialGamePiece(UnifiedArmSubsystem.GamePieceMode.Cone);

            setSecondGamePiece(UnifiedArmSubsystem.GamePieceMode.Cube);
            //setSecondScoringMode(ScoringMode.High);
            //setSecondScoringLocationIndex(8);

            setMantlePrepPosition(MantlePrepPosition.OutsideCommunity);

            setEnableDrivePhaseOne(true);
            setEnableAcquireGamePiece(true);
            setEnableMoveToScore(false);
            setEnableSecondScore(false);
            setEnableBalance(false);
        }).ignoringDisable(true);
    }

    /**
     * Bottom lane, score cone high, then collect (but don't attempt to score) a cube
     *
     * @return command to configure the autonomous mode
     */
    public WrapperCommand createFavoriteAutoThree() {
        return new NamedInstantCommand("OracleFavoriteThree", () -> {
            setLane(Lane.Bottom);
            setInitialScoringLocationIndex(1);
            setInitialScoringMode(ScoringMode.High);
            setInitialGamePiece(UnifiedArmSubsystem.GamePieceMode.Cone);

            setSecondGamePiece(UnifiedArmSubsystem.GamePieceMode.Cube);
            //setSecondScoringMode(ScoringMode.High);
            //setSecondScoringLocationIndex(8);

            setMantlePrepPosition(MantlePrepPosition.OutsideCommunity);

            setEnableDrivePhaseOne(true);
            setEnableAcquireGamePiece(true);
            setEnableMoveToScore(false);
            setEnableSecondScore(false);
            setEnableBalance(false);
        }).ignoringDisable(true);
    }

    /**
     * More ambitious top lane program. Score cone high, then collect and score a cube next to it.
     *
     * @return command to configure the autonomous mode
     */
    public WrapperCommand createFavoriteAutoFour() {
        return new NamedInstantCommand("OracleFavoriteFour", () -> {
            setLane(Lane.Top);
            setInitialScoringLocationIndex(9);
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
            setEnableBalance(false);
        }).ignoringDisable(true);
    }

    /**
     * More ambitious middle lane program. Score cone high, then get mobility and balance.
     *
     * @return command to configure the autonomous mode
     */
    public WrapperCommand createFavoriteAutoFive() {
        return new NamedInstantCommand("OracleFavoriteFive", () -> {
            setLane(Lane.Middle);
            setInitialScoringLocationIndex(4);
            setInitialScoringMode(ScoringMode.High);
            setInitialGamePiece(UnifiedArmSubsystem.GamePieceMode.Cone);

            //setSecondGamePiece(UnifiedArmSubsystem.GamePieceMode.Cube);
            //setSecondScoringMode(ScoringMode.High);
            //setSecondScoringLocationIndex(8);

            setMantlePrepPosition(MantlePrepPosition.OutsideCommunity);

            setEnableDrivePhaseOne(true);
            setEnableAcquireGamePiece(false);
            setEnableMoveToScore(false);
            setEnableSecondScore(false);
            setEnableBalance(false);
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

    private Pose2d createLandmarkJustBeforeScoringPosition(int scoringIndex, double inchesPulledBack) {
        var scoringPosition = getLocationForScoringPositionIndex(scoringIndex);
        return createAdjustedLandmark(scoringPosition, inchesPulledBack, 0);
    }

    private Pose2d createAdjustedLandmark(Pose2d landmark, double xAdjustment, double yAdjustment) {
        return new Pose2d(
                new Translation2d(
                        landmark.getTranslation().getX() + xAdjustment,
                        landmark.getTranslation().getY() + yAdjustment),
                landmark.getRotation()
        );
    }

    public boolean isAutoCustomized() {
        boolean drivingOrBalancing = enableDrivePhaseOne.get() || enableBalance.get();
        boolean secondGamePiece = enableAcquireGamePiece.get() || enableMoveToScore.get() || enableSecondScore.get();

        return drivingOrBalancing || secondGamePiece;
    }

    public void printAllAutoSettings() {
        log.info("Lane: " + lane.toString());
        log.info("Enables: " + enableDrivePhaseOne.get() + "," + enableAcquireGamePiece.get() + ","
                + enableMoveToScore.get() + "," + enableSecondScore.get() + "," + enableBalance.get());
        log.info("InitialGamepiece: " + initialScoringLocationIndex + "," + initialGamePiece + "," + initialScoringMode);
        log.info("SecondGamepiece: " + secondScoringLocationIndex + "," + secondGamePiece + "," + secondScoringMode);
        log.info("Mantling:" + mantlePrepPosition);
    }
}
