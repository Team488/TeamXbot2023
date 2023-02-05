package competition.subsystems.arm.cspace;

import org.junit.Ignore;
import org.junit.Test;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class CSpaceTest {

    @Test
    @Ignore
    public void testSimpleAngle() throws IOException {
        CSpaceGenerator generator = new CSpaceGenerator();

        List<String> lines = new ArrayList<>();
        for (int lowerArmAngle = 0; lowerArmAngle < 180; lowerArmAngle++) {
            for (int upperArmAngle = 0; upperArmAngle > -180; upperArmAngle--) {
                var result = generator.testAngles(lowerArmAngle, upperArmAngle);
                String line = lowerArmAngle + "," + upperArmAngle + "," + result;
                lines.add(line);
            }
        }
        // Write all lines out to a file called C:\FRC\2023\CSpace.csv

        FileWriter writer = new FileWriter("C:\\FRC\\2023\\CSpace.csv");
        for (String line : lines) {
            writer.write(line + System.lineSeparator());
        }

    }
}
