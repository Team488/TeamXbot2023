
package competition.auto_programs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class BluetoRedConversion {
    double fieldMidpoint = 325.0;
    public Pose2d convertBluetoRed(Pose2d blueCoordinates){
        double redXCoordinates = ((fieldMidpoint-blueCoordinates.getX()) * 2) + blueCoordinates.getX();
        Rotation2d heading = blueCoordinates.getRotation();
        Rotation2d convertedHeading = Rotation2d.fromDegrees(heading.getDegrees() - (heading.getDegrees() - 90.0) * 2);

        return new Pose2d(redXCoordinates, blueCoordinates.getY(),convertedHeading);
    }

}