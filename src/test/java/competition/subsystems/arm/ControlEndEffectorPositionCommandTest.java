package competition.subsystems.arm;

import competition.BaseCompetitionTest;
import competition.subsystems.arm.commands.ControlEndEffectorPositionCommand;
import competition.subsystems.arm.commands.UnifiedArmMaintainer;
import junitparams.JUnitParamsRunner;
import junitparams.Parameters;
import org.junit.Test;
import org.junit.runner.RunWith;
import xbot.common.controls.sensors.mock_adapters.MockDutyCycleEncoder;
import xbot.common.math.XYPair;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

@RunWith(JUnitParamsRunner.class)
public class ControlEndEffectorPositionCommandTest extends BaseCompetitionTest {

    private ControlEndEffectorPositionCommand command;
    private UnifiedArmSubsystem arm;
    private UnifiedArmMaintainer maintainer;

    @Override
    public void setUp() {
        super.setUp();
        command = getInjectorComponent().controlEndEffectorPositionCommand();
        arm = getInjectorComponent().unifiedArmSubsystem();
        arm.setIsCalibrated(true);
        maintainer = getInjectorComponent().unifiedArmMaintainer();

        arm.upperArm.setAbsoluteEncoderOffsetInDegrees(0);
        arm.lowerArm.setAbsoluteEncoderOffsetInDegrees(0);
    }

    @Test
    public void testMoveArmWithCommand() {
        setArmAngles(45, 90);
        XYPair currentPosition = arm.getCurrentXZCoordinates();
        assertEquals(56.92, currentPosition.x, 0.01);
        assertEquals(6.01, currentPosition.y, 0.01);

        command.setDirection(new XYPair(-1, 0));
        command.initialize();
        command.execute();

        setArmAngles(arm.getTargetValue().x, arm.getTargetValue().y);
        maintainer.execute();
        timer.advanceTimeInSecondsBy(10);
        maintainer.execute();

        assertTrue(arm.isMaintainerAtGoal());
        command.end(false);

        assertEquals(45.45, arm.lowerArm.getArmPositionInDegrees(), 0.01);
        assertEquals(88.99, arm.upperArm.getArmPositionInDegrees(), 0.01);
    }

    @Test
    public void testMoveArmWithCommand2() {
        setArmAngles(95, 56);
        XYPair currentPosition = arm.getCurrentXZCoordinates();
        assertEquals(27.61, currentPosition.x, 0.01);
        assertEquals(26.88, currentPosition.y, 0.01);

        command.setDirection(new XYPair(0, 1));
        command.initialize();
        command.execute();

        setArmAngles(arm.getTargetValue().x, arm.getTargetValue().y);
        maintainer.execute();
        timer.advanceTimeInSecondsBy(10);
        maintainer.execute();

        assertTrue(arm.isMaintainerAtGoal());
        command.end(false);

        assertEquals(95.36, arm.lowerArm.getArmPositionInDegrees(), 0.01);
        assertEquals(56.58, arm.upperArm.getArmPositionInDegrees(), 0.01);
    }

    @Test
    public void testMoveArmWithCommandBackwards() {
        setArmAngles(86, -88);
        XYPair currentPosition = arm.getCurrentXZCoordinates();
        assertEquals(-32.87, currentPosition.x, 0.01);
        assertEquals(45.65, currentPosition.y, 0.01);

        command.setDirection(new XYPair(0, 1));
        command.initialize();
        command.execute();

        setArmAngles(arm.getTargetValue().x, arm.getTargetValue().y);
        maintainer.execute();
        timer.advanceTimeInSecondsBy(10);
        maintainer.execute();

        assertTrue(arm.isMaintainerAtGoal());
        command.end(false);

        assertEquals(86.03, arm.lowerArm.getArmPositionInDegrees(), 0.01);
        assertEquals(-88.82, arm.upperArm.getArmPositionInDegrees(), 0.01);
    }

    @Test
    public void testMoveArmWithCommandExceedingLimits() {
        // Starting position is illegal. We will snap to the closest legal position.
        setArmAngles(20, 170);
        XYPair currentPosition = arm.getCurrentXZCoordinates();
        assertEquals(77.27, currentPosition.x, 0.01);
        assertEquals(21.47, currentPosition.y, 0.01);

        command.setDirection(new XYPair(-1, 0));
        command.initialize();
        command.execute();

        setArmAngles(arm.getTargetValue().x, arm.getTargetValue().y);
        maintainer.execute();
        timer.advanceTimeInSecondsBy(10);
        maintainer.execute();

        assertTrue(arm.isMaintainerAtGoal());
        command.end(false);

        assertEquals(51.66, arm.lowerArm.getArmPositionInDegrees(), 0.01);
        assertEquals(106.42, arm.upperArm.getArmPositionInDegrees(), 0.01);
        XYPair newPosition = arm.getCurrentXZCoordinates();
        assertEquals(61, newPosition.x, 0.01);
        assertEquals(21.47, newPosition.y, 0.01);
    }

    @Test
    @Parameters({
            "35, -50",
            "35, 45",
            "86, -88",
            "95, 70"
    })
    public void testMoveArmWithCommandHoldUp(double startingLowerAngle, double startingUpperAngle) {
        startingLowerAngle = arm.lowerArm.coerceAngleWithinLimits(startingLowerAngle);
        startingUpperAngle = arm.upperArm.coerceAngleWithinLimits(startingUpperAngle);

        setArmAngles(startingLowerAngle, startingUpperAngle);
        XYPair initialPosition = arm.getCurrentXZCoordinates();

        command.setDirection(new XYPair(0, 1));
        command.initialize();

        XYPair previousPosition = initialPosition;

        // Simulate holding the command for a while
        for (int i = 0; i < 1000; i++) {
            command.execute();
            setArmAngles(arm.getTargetValue().x, arm.getTargetValue().y);
            maintainer.execute();
            timer.advanceTimeInSecondsBy(0.1);
            maintainer.execute();

            // Check that movement continues upwards
            XYPair currentPosition = arm.getCurrentXZCoordinates();
            assertEquals(previousPosition.x, currentPosition.x, 1);
            assertTrue(currentPosition.y >= previousPosition.y);
            previousPosition = currentPosition;

            if (command.isFinished()) {
                break;
            }
        }

        assertTrue(arm.isMaintainerAtGoal());
        command.end(false);

        XYPair finalPosition = arm.getCurrentXZCoordinates();
        assertEquals(initialPosition.x, finalPosition.x, 1);
        assertTrue(finalPosition.y <= arm.getMaximumZPosition());

        // Check that the upper arm did not cross the lower arm
        assertTrue(Math.abs(arm.getCurrentValue().y + startingUpperAngle) >= Math.abs(startingUpperAngle));
    }

    private void setArmAngles(double lowerArmAngle, double upperArmAngle) {
        ((MockDutyCycleEncoder)arm.lowerArm.absoluteEncoder).setRawPosition(lowerArmAngle/360.0);
        ((MockDutyCycleEncoder)arm.upperArm.absoluteEncoder).setRawPosition(upperArmAngle/360.0);
    }
}
