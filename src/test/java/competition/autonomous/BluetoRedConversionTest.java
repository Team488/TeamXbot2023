package competition.autonomous;
import competition.auto_programs.BluetoRedConversion;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertSame;

public class BluetoRedConversionTest{
    @Test
    public void testSimpleCoordinate(){
        BluetoRedConversion converter = new BluetoRedConversion();

        Pose2d redPose = converter.convertBluetoRed(new Pose2d(168,1, Rotation2d.fromDegrees(135)));
        assertEquals(482, redPose.getX(), 0.001);
        assertEquals(1, redPose.getY(), 0.001);
        assertEquals(45, redPose.getRotation().getDegrees(), 0.001);

        Pose2d redPosePositiveExtremes = converter.convertBluetoRed(new Pose2d(1000000,4032, Rotation2d.fromDegrees(360)));
        assertEquals(-999350, redPosePositiveExtremes.getX(), 0.001);
        assertEquals(4032, redPosePositiveExtremes.getY(), 0.001);
        assertEquals(-180, redPosePositiveExtremes.getRotation().getDegrees(), 0.001);

        Pose2d redPoseNegativeExtremes = converter.convertBluetoRed(new Pose2d(-1000000,-4032, Rotation2d.fromDegrees(-360)));
        assertEquals(1000650, redPoseNegativeExtremes.getX(), 0.001);
        assertEquals(-4032, redPoseNegativeExtremes.getY(), 0.001);
        assertEquals(540, redPoseNegativeExtremes.getRotation().getDegrees(), 0.001);

        Pose2d redPoseUpperBoundaries = converter.convertBluetoRed(new Pose2d(650,315, Rotation2d.fromDegrees(180)));
        assertEquals(0, redPoseUpperBoundaries.getX(), 0.001);
        assertEquals(315, redPoseUpperBoundaries.getY(), 0.001);
        assertEquals(0, redPoseUpperBoundaries.getRotation().getDegrees(), 0.001);

        Pose2d redPoseLowerBoundaries = converter.convertBluetoRed(new Pose2d(0,0, Rotation2d.fromDegrees(0)));
        assertEquals(650, redPoseLowerBoundaries.getX(), 0.001);
        assertEquals(0, redPoseLowerBoundaries.getY(), 0.001);
        assertEquals(180, redPoseLowerBoundaries.getRotation().getDegrees(), 0.001);

        Pose2d redPoseAroundBoundariesMinusOne = converter.convertBluetoRed(new Pose2d(649,1, Rotation2d.fromDegrees(179)));
        assertEquals(1, redPoseAroundBoundariesMinusOne.getX(), 0.001);
        assertEquals(1, redPoseAroundBoundariesMinusOne.getY(), 0.001);
        assertEquals(1, redPoseAroundBoundariesMinusOne.getRotation().getDegrees(), 0.001);

        Pose2d redPoseAroundBoundariesPlusOne = converter.convertBluetoRed(new Pose2d(1,314, Rotation2d.fromDegrees(1)));
        assertEquals(649, redPoseAroundBoundariesPlusOne.getX(), 0.001);
        assertEquals(314, redPoseAroundBoundariesPlusOne.getY(), 0.001);
        assertEquals(179, redPoseAroundBoundariesPlusOne.getRotation().getDegrees(), 0.001);


    }
}