package competition.subsystems.arm;

import competition.BaseCompetitionTest;
import competition.subsystems.arm.commands.ControlArmsWithJoyStickCommand_MembersInjector;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.Ignore;
import org.junit.Test;
import xbot.common.controls.actuators.mock_adapters.MockCANSparkMax;
import xbot.common.math.XYPair;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

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

    @Test
    public void testBrakes() {
        arms.setBrake(true);
        // The brakes are engaged by default, which means they are the "de-energized" state on
        // the actual solenoid.
        assertTrue(arms.areBrakesEngaged.get());
        assertFalse(arms.lowerArmBrakeSolenoid.getAdjusted());

        arms.setBrake(false);
        assertTrue(!arms.areBrakesEngaged.get());
        assertTrue(arms.lowerArmBrakeSolenoid.getAdjusted());
    }

    @Test
    public void testSetPowerWithBrake() {
        arms.setPower(new XYPair(1, 1));
        checkArmPowers(1, 1);

        arms.setBrake(true);
        arms.setPower(new XYPair(0.5, .75));
        // When the brake is engaged, the lower arm power should be forced to zero.
        checkArmPowers(0, .75);
    }

    @Test
    public void testSetAnglesWithBrake() {
        assertEquals(0, arms.lowerArm.getArmPositionInDegrees(), 0.001);
        assertEquals(0, arms.upperArm.getArmPositionInDegrees(), 0.001);
        arms.setPower(new XYPair(1, 1));

        arms.setBrake(true);
        arms.setArmsToAngles(Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(-90));
        // because the brake is engaged, the lower arm will not use reference and will instead set power
        // to 0.
        assertEquals(0, arms.lowerArm.rightMotor.get(), 0.001);
        // The actual reference setting logic is complex, but genreally speaking we should see a large
        // number of "negative rotations" as a reference.
        assertTrue(((MockCANSparkMax)arms.upperArm.rightMotor).getReference() < -10);
    }

    @Test
    public void testMirrorArmAngles() {
        XYPair perfectlyVertical = new XYPair(90, 0);
        XYPair mirroredVertical = UnifiedArmSubsystem.mirrorArmAngles(perfectlyVertical);
        // There should be no change if we mirror around the center point of 90 for the lower arm and -90 for the upper.
        assertEquals(90, mirroredVertical.x, 0.001);
        assertEquals(0, mirroredVertical.y, 0.001);

        // We should see some changes using angles offset by 45 degrees.
        XYPair offset = new XYPair(135, -45);
        XYPair mirroredOffset = UnifiedArmSubsystem.mirrorArmAngles(offset);
        assertEquals(45, mirroredOffset.x, 0.001);
        assertEquals(45, mirroredOffset.y, 0.001);

        // Let's check some angles that are more than 135 degrees away from the mirror point.
        XYPair farAway = new XYPair(-45, 45);
        XYPair mirroredFarAway = UnifiedArmSubsystem.mirrorArmAngles(farAway);
        assertEquals(225, mirroredFarAway.x, 0.001);
        assertEquals(-45, mirroredFarAway.y, 0.001);
    }

    private void checkArmPowers(double lowerPower, double upperPower) {
        assertEquals(lowerPower, arms.lowerArm.rightMotor.get(), 0.001);
        assertEquals(upperPower, arms.upperArm.rightMotor.get(), 0.001);
    }
}
