package competition.subsystems.arm;

import competition.BaseCompetitionTest;
import competition.subsystems.arm.commands.SimpleSafeArmRouterCommand;
import competition.subsystems.arm.commands.UnifiedArmMaintainer;
import org.junit.Test;
import xbot.common.controls.sensors.mock_adapters.MockDutyCycleEncoder;
import xbot.common.math.XYPair;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class SimpleSafeArmRouterCommandTest extends BaseCompetitionTest {

    private SimpleSafeArmRouterCommand routerCommand;
    private UnifiedArmSubsystem arms;
    private UnifiedArmMaintainer maintainer;

    @Override
    public void setUp() {
        super.setUp();
        arms = getInjectorComponent().unifiedArmSubsystem();
        arms.setIsCalibrated(true);
        maintainer = getInjectorComponent().unifiedArmMaintainer();
        routerCommand = getInjectorComponent().simpleSafeArmRouterCommand();
    }

    @Test
    public void testSimpleRoute() {
        // Same facing, just going from neutral state to high state.
        setArmAngles(85, -90);

        // -------------------------------------------
        // Going to first transition point
        // -------------------------------------------
        arms.refreshDataFrame();
        routerCommand.setTarget(UnifiedArmSubsystem.KeyArmPosition.HighGoal, UnifiedArmSubsystem.RobotFacing.Forward);

        routerCommand.initialize();
        // Should be going to same-side transition point
        checkCurrentTarget(UnifiedArmSubsystem.KeyArmPosition.SafeExternalTransition, UnifiedArmSubsystem.RobotFacing.Forward);
        routerCommand.execute();
        // Nothing should change on first execution.
        checkCurrentTarget(UnifiedArmSubsystem.KeyArmPosition.SafeExternalTransition, UnifiedArmSubsystem.RobotFacing.Forward);
        assertFalse(routerCommand.isFinished());

        maintainer.initialize();
        maintainer.execute();

        // Run the router again - since no change in the arm, we shouldn't have any change in the target.
        routerCommand.execute();
        checkCurrentTarget(UnifiedArmSubsystem.KeyArmPosition.SafeExternalTransition, UnifiedArmSubsystem.RobotFacing.Forward);
        assertFalse(routerCommand.isFinished());

        // Now move the arm to the transition point.
        setArmAnglesFromKeyArmPositionAndFacing(UnifiedArmSubsystem.KeyArmPosition.SafeExternalTransition, UnifiedArmSubsystem.RobotFacing.Forward);
        arms.refreshDataFrame();
        maintainer.execute();
        // We'll need to advance time and maintain again to get the "is at target" flag to be set.
        assertFalse(arms.isMaintainerAtGoal());
        timer.advanceTimeInSecondsBy(10);
        arms.refreshDataFrame();
        maintainer.execute();
        assertTrue(arms.isMaintainerAtGoal());

        // -------------------------------------------
        // Going to goal point
        // -------------------------------------------

        // This should cause the router to move to the next target.
        routerCommand.execute();
        checkCurrentTarget(UnifiedArmSubsystem.KeyArmPosition.HighGoal, UnifiedArmSubsystem.RobotFacing.Forward);
        assertFalse(routerCommand.isFinished());

        arms.refreshDataFrame();
        maintainer.execute();
        // Now move the arm to the high goal point.
        setArmAnglesFromKeyArmPositionAndFacing(UnifiedArmSubsystem.KeyArmPosition.HighGoal, UnifiedArmSubsystem.RobotFacing.Forward);
        arms.refreshDataFrame();
        maintainer.execute();
        assertFalse(arms.isMaintainerAtGoal());
        // We'll need to advance time and maintain again to get the "is at target" flag to be set.
        timer.advanceTimeInSecondsBy(10);
        arms.refreshDataFrame();
        maintainer.execute();
        assertTrue(arms.isMaintainerAtGoal());

        // This should cause the router to remove the target and declare it is finished.
        routerCommand.execute();
        assertTrue(routerCommand.getArmPosesToVisit().size() == 0);
        assertTrue(routerCommand.isFinished());
    }

    @Test
    public void testCrossFacingRoute() {
        // Same facing, just going from neutral state to high state.
        setArmAngles(85, -90);
        arms.refreshDataFrame();
        // -------------------------------------------
        // Going to first transition point (forward)
        // -------------------------------------------

        routerCommand.setTarget(UnifiedArmSubsystem.KeyArmPosition.HighGoal, UnifiedArmSubsystem.RobotFacing.Backward);

        routerCommand.initialize();
        // Should be going to same-side transition point
        checkCurrentTarget(UnifiedArmSubsystem.KeyArmPosition.SafeExternalTransition, UnifiedArmSubsystem.RobotFacing.Forward);
        routerCommand.execute();
        // Nothing should change on first execution.
        checkCurrentTarget(UnifiedArmSubsystem.KeyArmPosition.SafeExternalTransition, UnifiedArmSubsystem.RobotFacing.Forward);
        assertFalse(routerCommand.isFinished());

        arms.refreshDataFrame();
        maintainer.initialize();
        maintainer.execute();

        // Run the router again - since no change in the arm, we shouldn't have any change in the target.
        routerCommand.execute();
        checkCurrentTarget(UnifiedArmSubsystem.KeyArmPosition.SafeExternalTransition, UnifiedArmSubsystem.RobotFacing.Forward);
        assertFalse(routerCommand.isFinished());

        // Now move the arm to the transition point.
        setArmAnglesFromKeyArmPositionAndFacing(UnifiedArmSubsystem.KeyArmPosition.SafeExternalTransition, UnifiedArmSubsystem.RobotFacing.Forward);
        arms.refreshDataFrame();
        maintainer.execute();
        // We'll need to advance time and maintain again to get the "is at target" flag to be set.
        assertFalse(arms.isMaintainerAtGoal());
        timer.advanceTimeInSecondsBy(10);
        arms.refreshDataFrame();
        maintainer.execute();
        assertTrue(arms.isMaintainerAtGoal());

        // -------------------------------------------
        // Going to second transition point (backward)
        // -------------------------------------------

        // This should cause the router to move to the next target.
        routerCommand.execute();
        checkCurrentTarget(UnifiedArmSubsystem.KeyArmPosition.SafeExternalTransition, UnifiedArmSubsystem.RobotFacing.Backward);
        assertFalse(routerCommand.isFinished());

        arms.refreshDataFrame();
        maintainer.execute();
        // Now move the arm to the transition point.
        setArmAnglesFromKeyArmPositionAndFacing(UnifiedArmSubsystem.KeyArmPosition.SafeExternalTransition, UnifiedArmSubsystem.RobotFacing.Backward);
        arms.refreshDataFrame();
        maintainer.execute();
        // We'll need to advance time and maintain again to get the "is at target" flag to be set.
        assertFalse(arms.isMaintainerAtGoal());
        timer.advanceTimeInSecondsBy(10);
        arms.refreshDataFrame();
        maintainer.execute();
        assertTrue(arms.isMaintainerAtGoal());

        // -------------------------------------------
        // Going to goal point
        // -------------------------------------------

        // This should cause the router to move to the next target.
        routerCommand.execute();
        checkCurrentTarget(UnifiedArmSubsystem.KeyArmPosition.HighGoal, UnifiedArmSubsystem.RobotFacing.Backward);
        assertFalse(routerCommand.isFinished());

        arms.refreshDataFrame();
        maintainer.execute();
        // Now move the arm to the high goal point.
        setArmAnglesFromKeyArmPositionAndFacing(UnifiedArmSubsystem.KeyArmPosition.HighGoal, UnifiedArmSubsystem.RobotFacing.Backward);
        arms.refreshDataFrame();
        maintainer.execute();
        // We'll need to advance time and maintain again to get the "is at target" flag to be set.
        timer.advanceTimeInSecondsBy(10);
        arms.refreshDataFrame();
        maintainer.execute();
        assertTrue(arms.isMaintainerAtGoal());

        // This should cause the router to remove the target and declare it is finished.
        routerCommand.execute();
        assertTrue(routerCommand.getArmPosesToVisit().size() == 0);
        assertTrue(routerCommand.isFinished());
    }

    @Test
    public void testSafeExternalTransitionAsGoal() {
        setArmAngles(85, -90);
        routerCommand.setTarget(UnifiedArmSubsystem.KeyArmPosition.SafeExternalTransition, UnifiedArmSubsystem.RobotFacing.Backward);
        routerCommand.initialize();

        assertTrue(routerCommand.getArmPosesToVisit().size() == 2);
        checkCurrentTarget(UnifiedArmSubsystem.KeyArmPosition.SafeExternalTransition, UnifiedArmSubsystem.RobotFacing.Forward);
        checkArbitraryTarget(UnifiedArmSubsystem.KeyArmPosition.SafeExternalTransition, UnifiedArmSubsystem.RobotFacing.Backward, 1);
    }

    private void setArmAngles(double lowerArmAngle, double upperArmAngle) {
        ((MockDutyCycleEncoder)arms.lowerArm.absoluteEncoder).setRawPosition(lowerArmAngle/360.0);
        ((MockDutyCycleEncoder)arms.upperArm.absoluteEncoder).setRawPosition(upperArmAngle/360.0);
    }

    private void setArmAnglesFromKeyArmPositionAndFacing(UnifiedArmSubsystem.KeyArmPosition position, UnifiedArmSubsystem.RobotFacing facing) {
        XYPair angles = arms.getKeyArmAngles(position, facing);
        setArmAngles(angles.x, angles.y);
    }

    private void checkCurrentTarget(UnifiedArmSubsystem.KeyArmPosition position, UnifiedArmSubsystem.RobotFacing facing) {
        checkArbitraryTarget(position, facing, 0);
    }

    private void checkArbitraryTarget(UnifiedArmSubsystem.KeyArmPosition position, UnifiedArmSubsystem.RobotFacing facing, int index) {
        assertEquals(position, routerCommand.getArmPosesToVisit().get(index).getFirst());
        assertEquals(facing, routerCommand.getArmPosesToVisit().get(index).getSecond());
    }
}
