package competition.subsystems.arm;

import com.sun.source.tree.AssertTree;
import competition.BaseCompetitionTest;
import competition.subsystems.arm.commands.UnifiedArmMaintainer;
import org.junit.Test;
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
    }

    @Test
    public void testBrakeChangeState() {
        assertTrue(arms.areBrakesEngaged());
        assertEquals(0, arms.getCurrentValue().x, 0.001);
        assertEquals(0, arms.getCurrentValue().y, 0.001);
        arms.setTargetValue(new XYPair(90, -90));
        maintainer.execute();

        // Since we are very far from our target, the brake should disengage.
        assertFalse(arms.areBrakesEngaged());

        // Now, let's move to a position where the brakes should engage.
        arms.setTargetValue(new XYPair(maintainer.getLowerArmErrorThresholdToEngageBrake() - 0.1, -90));
        maintainer.execute();

        // Since we are very close to our target, but this is just the first time we've been close, the brakes should not engage.
        assertFalse(arms.areBrakesEngaged());

        // Wait a while
        timer.advanceTimeInSecondsBy(2);

        // Now that we've been close for a while, the brakes should engage.
        maintainer.execute();
        assertTrue(arms.areBrakesEngaged());

        // To test hysterisis, move the position just outside the threshold.
        arms.setTargetValue(new XYPair(maintainer.getLowerArmErrorThresholdToDisengageBrake() - 0.1, -90));
        maintainer.execute();

        // Even though we are outside our brake engage threshold, we haven't moved outside the disengage threshold.
        assertTrue(arms.areBrakesEngaged());

        // Now, move outside the disengage threshold.
        arms.setTargetValue(new XYPair(maintainer.getLowerArmErrorThresholdToDisengageBrake() + 0.1, -90));
        maintainer.execute();

        // Now that we are outside the disengage threshold, the brakes should disengage.
        assertFalse(arms.areBrakesEngaged());
    }
}
