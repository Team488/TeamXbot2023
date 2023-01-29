package competition.auto_programs;

public class BluetoRedConversion {
    double fieldMidpoint = 325.0;
    public double convertBluetoRed(double blueCoordinates){
        double redCoordinates = ((fieldMidpoint-blueCoordinates) * 2) + blueCoordinates;
        return redCoordinates;
    }
}
