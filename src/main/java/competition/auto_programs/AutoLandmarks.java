package competition.auto_programs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

import javax.inject.Inject;

public class AutoLandmarks {
    @Inject
    public AutoLandmarks() { }

    // Every game piece coordinate
    public static Pose2d blueGamePieceUpper = new Pose2d(261,182, Rotation2d.fromDegrees(0));
    public static Pose2d blueGamePieceSecondUpper = new Pose2d(261,131, Rotation2d.fromDegrees(0));
    public static Pose2d blueGamePieceLower = new Pose2d(261,36, Rotation2d.fromDegrees(0));
    public static Pose2d blueGamePieceSecondLower = new Pose2d(261,84, Rotation2d.fromDegrees(0));

    // Every checkpoint coordinate
    public static Pose2d blueLowerCommunitySideMidCheckpoint = new Pose2d(87,29, Rotation2d.fromDegrees(-180));
    public static Pose2d blueLowerGamePieceSideMidCheckpoint = new Pose2d(220,29, Rotation2d.fromDegrees(-180));
    public static Pose2d blueUpperCommunitySideMidCheckpoint = new Pose2d(87,178, Rotation2d.fromDegrees(-180));
    public static Pose2d blueUpperGamePieceSideMidCheckpoint = new Pose2d(220,195, Rotation2d.fromDegrees(-180));

    // Charge station checkpoints + charge station coordinate
    public static Pose2d blueChargeStationCenter = new Pose2d(157,110, Rotation2d.fromDegrees(-180));
    public static Pose2d blueChargeStationMantleFromLeft = new Pose2d(140,110, Rotation2d.fromDegrees(-180));
    public static Pose2d blueChargeStationMantleFromRight = new Pose2d(170,110, Rotation2d.fromDegrees(-180));
    public static Pose2d blueBelowChargeStation = new Pose2d(170,30, Rotation2d.fromDegrees(-180));

    public static Pose2d blueToUpperAndLowerCommunityCheckpoint = new Pose2d(87,110, Rotation2d.fromDegrees(-180));
    public static Pose2d blueToUpperAndLowerFieldCheckpoint = new Pose2d(239,110, Rotation2d.fromDegrees(-180));

    public static Pose2d blueUpperCheckpointOutsideCommunity = new Pose2d(239, 178, Rotation2d.fromDegrees(-180));
    public static Pose2d blueLowerCheckpointOutsideCommunity = new Pose2d(239, 29, Rotation2d.fromDegrees(-180));

    // Every single blue scoring position
    // Each position is 22in from the next
    public static Pose2d blueScoringPositionOne = new Pose2d(66, 20, Rotation2d.fromDegrees(-180));
    public static Pose2d blueScoringPositionTwo = new Pose2d(66, 42, Rotation2d.fromDegrees(-180));
    public static Pose2d blueScoringPositionThree = new Pose2d(66, 64, Rotation2d.fromDegrees(-180));
    public static Pose2d blueScoringPositionFour = new Pose2d(66, 86, Rotation2d.fromDegrees(-180));
    public static Pose2d blueScoringPositionFive = new Pose2d(66, 108, Rotation2d.fromDegrees(-180));
    public static Pose2d blueScoringPositionSix = new Pose2d(66, 130, Rotation2d.fromDegrees(-180));
    public static Pose2d blueScoringPositionSeven = new Pose2d(66, 152, Rotation2d.fromDegrees(-180));
    public static Pose2d blueScoringPositionEight = new Pose2d(66, 174, Rotation2d.fromDegrees(-180));
    public static Pose2d blueScoringPositionNine = new Pose2d(66, 196, Rotation2d.fromDegrees(-180));

    // Other useful landmarks
    public static Pose2d blueNorthOfChargingStationOutsideCommunity = new Pose2d(185, 180, Rotation2d.fromDegrees(0));

    private static Pose2d convertBluetoRed(Pose2d blueCoordinates){
        double fieldMidpoint = 325.0;
        double redXCoordinates = ((fieldMidpoint-blueCoordinates.getX()) * 2) + blueCoordinates.getX();
        Rotation2d heading = blueCoordinates.getRotation();
        Rotation2d convertedHeading = Rotation2d.fromDegrees(heading.getDegrees() - (heading.getDegrees() - 90.0) * 2);

        return new Pose2d(redXCoordinates, blueCoordinates.getY(),convertedHeading);
    }

    public static Pose2d convertBlueToRedIfNeeded(Pose2d blueCoordinates) {
        return convertBlueToRedIfNeeded(blueCoordinates, DriverStation.getAlliance());
    }

    public static Pose2d convertBlueToRedIfNeeded(Pose2d blueCoordinates, DriverStation.Alliance alliance) {
        if (alliance == DriverStation.Alliance.Red) {
            return convertBluetoRed(blueCoordinates);
        }
        return blueCoordinates;
    }
}
