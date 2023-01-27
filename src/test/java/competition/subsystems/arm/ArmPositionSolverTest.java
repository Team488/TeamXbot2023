package competition.subsystems.arm;

import competition.BaseCompetitionTest;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

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
    }
}
