package competition.subsystems.arm;

import competition.BaseCompetitionTest;
import competition.subsystems.arm.commands.SimpleSafeArmRouterCommand;
import competition.subsystems.arm.commands.SimpleXZRouterCommand;
import competition.subsystems.arm.commands.UnifiedArmMaintainer;
import competition.trajectory.XbotArmPoint;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.Test;
import xbot.common.controls.sensors.mock_adapters.MockDutyCycleEncoder;
import xbot.common.math.XYPair;

import static org.junit.Assert.assertEquals;

public class SimpleXZRouterCommandTest extends BaseCompetitionTest {

    private SimpleXZRouterCommand routerCommand;
    private UnifiedArmSubsystem arms;
    private UnifiedArmMaintainer maintainer;

    @Override
    public void setUp() {
        super.setUp();
        arms = getInjectorComponent().unifiedArmSubsystem();
        arms.setIsCalibrated(true);
        maintainer = getInjectorComponent().unifiedArmMaintainer();
        routerCommand = getInjectorComponent().simpleXZRouterCommand();
        arms.upperArm.setAbsoluteEncoderOffsetInDegrees(0);
        arms.lowerArm.setAbsoluteEncoderOffsetInDegrees(0);
    }

    @Test
    public void testForwardPathGeneration() {
        setArmAngles(93, 5);

        routerCommand.setKeyPoint(new XbotArmPoint(new Translation2d(40,40), 0.5));
        routerCommand.initialize();
        var points = routerCommand.getPointsToInterpolate();
        // Since we started at the origin, it will need to visit all 3 safe points before it can go to the target,
        // so we should have 4 points total
        assertEquals(5, points.size(), 0.001);
    }

    @Test
    public void testDirectPathGeneration() {
        setArmAnglesForPoint(new Translation2d(60, 30));
        routerCommand.setKeyPoint(new XbotArmPoint(new Translation2d(70,40), 0.5));
        routerCommand.initialize();
        var points = routerCommand.getPointsToInterpolate();

        // This should be a direct route, so only one point.
        assertEquals(1, points.size(), 0.001);
    }



    private void setArmAnglesForPoint(Translation2d point) {
        var angles = arms.solver.solveArmJointPositions(
                new XYPair(point.getX(), point.getY()),
                arms.getCurrentValue());
        setArmAngles(angles.getLowerJointRotation().getDegrees(), angles.getUpperJointRotation().getDegrees());
    }

    private void setArmAngles(double lowerArmAngle, double upperArmAngle) {
        ((MockDutyCycleEncoder)arms.lowerArm.absoluteEncoder).setRawPosition(lowerArmAngle/360.0);
        ((MockDutyCycleEncoder)arms.upperArm.absoluteEncoder).setRawPosition(upperArmAngle/360.0);
    }
}
