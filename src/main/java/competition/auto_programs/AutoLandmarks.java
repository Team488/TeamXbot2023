package competition.auto_programs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AutoLandmarks {
    // Every game piece coordinate
    public Pose2d blueGamePieceUpper = new Pose2d(261,182, Rotation2d.fromDegrees(0));
    public Pose2d blueGamePieceSecondUpper = new Pose2d(261,131, Rotation2d.fromDegrees(0));
    public Pose2d blueGamePieceLower = new Pose2d(261,36, Rotation2d.fromDegrees(0));
    public Pose2d blueGamePieceSecondLower = new Pose2d(261,84, Rotation2d.fromDegrees(0));
    // Every checkpoint coordinate
    public Pose2d blueLowerCommunitySideMidCheckpoint = new Pose2d(87,29, Rotation2d.fromDegrees(-180));
    public Pose2d blueLowerGamePieceSideMidCheckpoint = new Pose2d(220,29, Rotation2d.fromDegrees(-180));
    public Pose2d blueUpperCommunitySideMidCheckpoint = new Pose2d(87,178, Rotation2d.fromDegrees(-180));
    public Pose2d blueUpperGamePieceSideMidCheckpoint = new Pose2d(220,195, Rotation2d.fromDegrees(-180));
    // Charge station checkpoints + charge station coordinate
    public Pose2d blueChargeStation = new Pose2d(150,110, Rotation2d.fromDegrees(-180));
    public Pose2d blueToUpperAndLowerCommunityCheckpoint = new Pose2d(87,110, Rotation2d.fromDegrees(-180));
    public Pose2d blueToUpperAndLowerFieldCheckpoint = new Pose2d(239,110, Rotation2d.fromDegrees(-180));


    // Every single blue scoring position
    public Pose2d blueScoringPositionOne = new Pose2d(66,16, Rotation2d.fromDegrees(-180));
    public Pose2d blueScoringPositionTwo = new Pose2d(66,40, Rotation2d.fromDegrees(-180));
    public Pose2d blueScoringPositionThree = new Pose2d(66,64, Rotation2d.fromDegrees(-180));
    public Pose2d blueScoringPositionFour = new Pose2d(66,85, Rotation2d.fromDegrees(-180));
    public Pose2d blueScoringPositionFive = new Pose2d(66,107, Rotation2d.fromDegrees(-180));
    public Pose2d blueScoringPositionSix = new Pose2d(66,130, Rotation2d.fromDegrees(-180));
    public Pose2d blueScoringPositionSeven = new Pose2d(66,152, Rotation2d.fromDegrees(-180));
    public Pose2d blueScoringPositionEight = new Pose2d(66,175, Rotation2d.fromDegrees(-180));
    public Pose2d blueScoringPositionNine = new Pose2d(66,200, Rotation2d.fromDegrees(-180));


}
