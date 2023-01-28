package competition.subsystems.arm;

import competition.BaseCompetitionTest;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class ArmPositionSolverTest extends BaseCompetitionTest {
    @Test
    public void testSolveDirectlyForwards() {
        ArmPositionSolverConfiguration configuration = new ArmPositionSolverConfiguration(
                1, 1,
                Rotation2d.fromDegrees(-180), Rotation2d.fromDegrees(0),
                Rotation2d.fromDegrees(-180), Rotation2d.fromDegrees(180));
        ArmPositionSolver solver = new ArmPositionSolver(configuration);

        ArmPositionState calculatedGoal = solver.solveArmJointPositions(2, 0);
        assertEquals(Rotation2d.fromDegrees(0), calculatedGoal.getLowerJointRotation());
        assertEquals(Rotation2d.fromDegrees(0), calculatedGoal.getUpperJointRotation());
        assertTrue(calculatedGoal.isSolveable());
    }

    @Test
    public void testSolveDirectlyBackwards() {
        ArmPositionSolverConfiguration configuration = new ArmPositionSolverConfiguration(
                1, 1,
                Rotation2d.fromDegrees(-180), Rotation2d.fromDegrees(0),
                Rotation2d.fromDegrees(-180), Rotation2d.fromDegrees(180));
        ArmPositionSolver solver = new ArmPositionSolver(configuration);

        ArmPositionState calculatedGoal = solver.solveArmJointPositions(-2, 0);
        assertEquals(Rotation2d.fromDegrees(180), calculatedGoal.getLowerJointRotation());
        assertEquals(Rotation2d.fromDegrees(180), calculatedGoal.getUpperJointRotation());
        assertTrue(calculatedGoal.isSolveable());
    }

    @Test
    public void testSolveDirectlyUpwards() {
        ArmPositionSolverConfiguration configuration = new ArmPositionSolverConfiguration(
                1, 1,
                Rotation2d.fromDegrees(-180), Rotation2d.fromDegrees(0),
                Rotation2d.fromDegrees(-180), Rotation2d.fromDegrees(180));
        ArmPositionSolver solver = new ArmPositionSolver(configuration);

        ArmPositionState calculatedGoal = solver.solveArmJointPositions(0, 2);
        assertEquals(Rotation2d.fromDegrees(90), calculatedGoal.getLowerJointRotation());
        assertEquals(Rotation2d.fromDegrees(90), calculatedGoal.getUpperJointRotation());
        assertTrue(calculatedGoal.isSolveable());
    }

    @Test
    public void testSolveHalfDistanceForwards() {
        ArmPositionSolverConfiguration configuration = new ArmPositionSolverConfiguration(
                1, 1,
                Rotation2d.fromDegrees(-180), Rotation2d.fromDegrees(0),
                Rotation2d.fromDegrees(-180), Rotation2d.fromDegrees(180));
        ArmPositionSolver solver = new ArmPositionSolver(configuration);

        ArmPositionState calculatedGoal = solver.solveArmJointPositions(1, 0);
        assertEquals(Rotation2d.fromDegrees(60), calculatedGoal.getLowerJointRotation());
        assertEquals(Rotation2d.fromDegrees(-60), calculatedGoal.getUpperJointRotation());
        assertTrue(calculatedGoal.isSolveable());
    }

    @Test
    public void testSolveImpossible() {
        ArmPositionSolverConfiguration configuration = new ArmPositionSolverConfiguration(
                1, 1,
                Rotation2d.fromDegrees(-180), Rotation2d.fromDegrees(0),
                Rotation2d.fromDegrees(-180), Rotation2d.fromDegrees(180));
        ArmPositionSolver solver = new ArmPositionSolver(configuration);

        ArmPositionState calculatedGoal = solver.solveArmJointPositions(3, 0);
        assertFalse(calculatedGoal.isSolveable());
    }
}
