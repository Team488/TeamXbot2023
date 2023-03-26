package competition.subsystems.drive;

import competition.BaseCompetitionTest;
import org.junit.Test;
import xbot.common.math.ContiguousDouble;

import static org.junit.Assert.assertEquals;

public class QuadrantTest extends BaseCompetitionTest {


    @Test
    public void testQuadrants() {
        assertEquals(0, getQuadrantAngle(0), 0.001);
        assertEquals(0, getQuadrantAngle(42), 0.001);
        assertEquals(0, getQuadrantAngle(44.999), 0.001);
        assertEquals(0, getQuadrantAngle(-10), 0.001);
        assertEquals(0, getQuadrantAngle(-44.999), 0.001);
        assertEquals(90, getQuadrantAngle(45.6), 0.001);
        assertEquals(90, getQuadrantAngle(110), 0.001);
        assertEquals(180, getQuadrantAngle(140), 0.001);
        assertEquals(180, getQuadrantAngle(140), 0.001);

        assertEquals(270, getQuadrantAngle(-60), 0.001);
        assertEquals(270, getQuadrantAngle(-110), 0.001);

        assertEquals(180, getQuadrantAngle(-170), 0.001);


    }

    private double getQuadrantAngle(double angle) {
        // First, we need to normalize the angle to be between -45 and 315
        double desiredHeading = ContiguousDouble.reboundValue(angle, -45, 315)+45;
        // Now, we can use the modulus operator to get the quadrant.
        int quadrant = (int) (desiredHeading / 90);
        return quadrant * 90;
    }
}
