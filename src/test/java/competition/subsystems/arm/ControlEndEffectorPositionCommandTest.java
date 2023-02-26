package competition.subsystems.arm;

import competition.BaseCompetitionTest;
import competition.subsystems.arm.commands.ControlEndEffectorPositionCommand;
import competition.subsystems.arm.commands.UnifiedArmMaintainer;
import org.junit.Test;
import xbot.common.controls.sensors.mock_adapters.MockDutyCycleEncoder;
import xbot.common.math.XYPair;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

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
    }

    @Test
    public void testMoveArmWithCommand() {
        setArmAngles(45, 90);
        XYPair currentPosition = arm.getCurrentXZCoordinates();
        assertEquals(54.80, currentPosition.x, 0.01);
        assertEquals(8.13, currentPosition.y, 0.01);

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
        assertEquals(88.94, arm.upperArm.getArmPositionInDegrees(), 0.01);
    }

    @Test
    public void testMoveArmWithCommand2() {
        setArmAngles(103, 56);
        XYPair currentPosition = arm.getCurrentXZCoordinates();
        assertEquals(20.79, currentPosition.x, 0.01);
        assertEquals(31.53, currentPosition.y, 0.01);

        command.setDirection(new XYPair(0, 1));
        command.initialize();
        command.execute();

        setArmAngles(arm.getTargetValue().x, arm.getTargetValue().y);
        maintainer.execute();
        timer.advanceTimeInSecondsBy(10);
        maintainer.execute();

        assertTrue(arm.isMaintainerAtGoal());
        command.end(false);

        assertEquals(103.27, arm.lowerArm.getArmPositionInDegrees(), 0.01);
        assertEquals(56.74, arm.upperArm.getArmPositionInDegrees(), 0.01);
    }

    @Test
    public void testMoveArmWithCommandBackwards() {
        setArmAngles(86, -88);
        XYPair currentPosition = arm.getCurrentXZCoordinates();
        assertEquals(-29.87, currentPosition.x, 0.01);
        assertEquals(45.54, currentPosition.y, 0.01);

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
        assertEquals(-88.89, arm.upperArm.getArmPositionInDegrees(), 0.01);
    }

    @Test
    public void testMoveArmWithCommandExceedingLimits() {
        // Starting position is illegal. We will snap to the closest legal position.
        setArmAngles(20, 170);
        XYPair currentPosition = arm.getCurrentXZCoordinates();
        assertEquals(74.31, currentPosition.x, 0.01);
        assertEquals(20.95, currentPosition.y, 0.01);

        command.setDirection(new XYPair(-1, 0));
        command.initialize();
        command.execute();

        setArmAngles(arm.getTargetValue().x, arm.getTargetValue().y);
        maintainer.execute();
        timer.advanceTimeInSecondsBy(10);
        maintainer.execute();

        assertTrue(arm.isMaintainerAtGoal());
        command.end(false);

        assertEquals(47.32, arm.lowerArm.getArmPositionInDegrees(), 0.01);
        assertEquals(111.80, arm.upperArm.getArmPositionInDegrees(), 0.01);
        XYPair newPosition = arm.getCurrentXZCoordinates();
        assertEquals(61, newPosition.x, 0.01);
        assertEquals(20.95, newPosition.y, 0.01);
    }

    private void setArmAngles(double lowerArmAngle, double upperArmAngle) {
        ((MockDutyCycleEncoder)arm.lowerArm.absoluteEncoder).setRawPosition(lowerArmAngle/360.0);
        ((MockDutyCycleEncoder)arm.upperArm.absoluteEncoder).setRawPosition(upperArmAngle/360.0);
    }
}
