package competition.subsystems.arm;

import competition.BaseCompetitionTest;
import org.junit.Ignore;
import org.junit.Test;
import xbot.common.math.XYPair;

import static org.junit.Assert.assertEquals;

public class UnifiedArmTest extends BaseCompetitionTest {

    UnifiedArmSubsystem arms;

    @Override
    public void setUp() {
        super.setUp();
        arms = getInjectorComponent().unifiedArmSubsystem();
        arms.setBrake(false);
    }

    @Test
    public void testSimplePower() {
        arms.setPower(new XYPair(0.5, .75));
        assertEquals(0.5, arms.lowerArm.rightMotor.get(), 0.001);
        assertEquals(0.75, arms.upperArm.rightMotor.get(), 0.001);
    }

    @Test
    public void testGetSetTarget() {
        arms.setTargetValue(new XYPair(0.5, .75));
        var target = arms.getTargetValue();
        assertEquals(0.5, target.x, 0.001);
        assertEquals(0.75, target.y, 0.001);
    }

    @Test
    @Ignore // Revisit once we set XZ positions rather than raw angles.
    public void testTypicalCalibrate() {
        arms.typicalCalibrate();
        var effectorPosition = arms.getCurrentValue();
        assertEquals(0, effectorPosition.x, 0.001);
        assertEquals(6, effectorPosition.y, 0.001); // Remember, this is "Z"
    }

    @Test
    @Ignore // Revisit once we set XZ positions rather than raw angles.
    public void testCalibrateAt() {
        arms.calibrateAt(90, 0);
        var effectorPosition = arms.getCurrentValue();
        assertEquals(32, effectorPosition.x, 0.001);
        assertEquals(38, effectorPosition.y, 0.001); // Remember, this is "Z"
    }
}
