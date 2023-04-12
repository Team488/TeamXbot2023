package competition.subsystems.arm;

import com.sun.source.tree.AssertTree;
import competition.BaseCompetitionTest;
import competition.subsystems.arm.commands.UnifiedArmMaintainer;
import org.junit.Ignore;
import org.junit.Test;
import xbot.common.controls.sensors.mock_adapters.MockDutyCycleEncoder;
import xbot.common.math.XYPair;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class UnifiedArmMaintainerTest extends BaseCompetitionTest {

    private UnifiedArmMaintainer maintainer;
    private UnifiedArmSubsystem arms;

    @Override
    public void setUp() {
        super.setUp();
        maintainer = getInjectorComponent().unifiedArmMaintainer();
        arms = getInjectorComponent().unifiedArmSubsystem();
        arms.setIsCalibrated(true);
        arms.upperArm.setAbsoluteEncoderOffsetInDegrees(0);
        arms.lowerArm.setAbsoluteEncoderOffsetInDegrees(0);

        setArmAngles(1,1);

    }

    @Test
    public void testBrakeChangeState() {
        assertTrue(arms.areBrakesEngaged());
        assertEquals(1, arms.getCurrentValue().x, 0.001);
        assertEquals(1, arms.getCurrentValue().y, 0.001);
        arms.setTargetValue(new XYPair(90, -90));
        maintainer.execute();

        // Since we are very far from our target, the brake should disengage.
        assertFalse(arms.areBrakesEngaged());

        setArmAngles(45, -90);
        // Now, let's move to a position where the brakes should engage.
        arms.setTargetValue(new XYPair(45+maintainer.getLowerArmErrorThresholdToEngageBrake() - 0.1, -90));
        maintainer.execute();

        // Since we are very close to our target, but this is just the first time we've been close, the brakes should not engage.
        assertFalse(arms.areBrakesEngaged());

        // Wait a while
        timer.advanceTimeInSecondsBy(2);

        // Now that we've been close for a while, the brakes should engage.
        maintainer.execute();
        assertTrue(arms.areBrakesEngaged());

        // To test hysterisis, move the position just outside the threshold.
        arms.setTargetValue(new XYPair(45+maintainer.getLowerArmErrorThresholdToDisengageBrake() - 0.1, -90));
        maintainer.execute();

        // Even though we are outside our brake engage threshold, we haven't moved outside the disengage threshold.
        assertTrue(arms.areBrakesEngaged());

        // Now, move outside the disengage threshold.
        arms.setTargetValue(new XYPair(45+maintainer.getLowerArmErrorThresholdToDisengageBrake() + 0.1, -90));
        maintainer.execute();

        // Now that we are outside the disengage threshold, the brakes should disengage.
        assertFalse(arms.areBrakesEngaged());
    }

    @Test
    public void testCubeModeExpandedErrorThreshold() {
        setArmAngles(45, 90);
        arms.setGamePieceMode(UnifiedArmSubsystem.GamePieceMode.Cone);
        arms.setTargetValue(new XYPair(45, 90));
        maintainer.execute();
        timer.advanceTimeInSecondsBy(2);
        maintainer.execute();
        assertTrue(arms.isMaintainerAtGoal());

        // disengage the brake
        arms.setTargetValue(new XYPair(90, 90));
        maintainer.execute();
        assertFalse(arms.areBrakesEngaged());
        // now actually test the situation

        arms.setTargetValue(new XYPair(45, 90));
        setArmAngles(42, 92);
        maintainer.execute();
        timer.advanceTimeInSecondsBy(0.25);
        maintainer.execute();
        assertFalse(arms.isMaintainerAtGoal());

        arms.setGamePieceMode(UnifiedArmSubsystem.GamePieceMode.Cube);
        maintainer.execute();
        timer.advanceTimeInSecondsBy(0.45);
        maintainer.execute();
        assertTrue(arms.isMaintainerAtGoal());

        var xypos = arms.getCurrentXZCoordinates();


    }



    private void setArmAngles(double lowerArmAngle, double upperArmAngle) {
        ((MockDutyCycleEncoder)arms.lowerArm.absoluteEncoder).setRawPosition(lowerArmAngle/360.0);
        ((MockDutyCycleEncoder)arms.upperArm.absoluteEncoder).setRawPosition(upperArmAngle/360.0);
    }
}
