package competition.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import xbot.common.math.XYPair;

public class ArmPositionSolver {
    private final ArmPositionSolverConfiguration configuration;

    public ArmPositionSolver(ArmPositionSolverConfiguration configuration) {
        this.configuration = configuration;
    }

    public ArmPositionSolverConfiguration getConfiguration() {
        return this.configuration;
    }

    /**
     * Calculate the target rotation of each arm joint given a target end-effector position
     * @param targetX The distance in front of or behind the robot (in inches) relative to the fixed arm pivot.
     * @param targetZ The distance above or below the fixed pivot (in inches)
     */
    public ArmPositionState solveArmJointPositions(double targetX, double targetZ) {
        // The arm is made up of two links. LinkA is fixed to a pivot point on the base of the robot (Joint1).
        // LinkA is a four-bar linkage. LinkB is attached to the other end of LinkA at Joint2. LinkA is a four-bar
        // linkage, so Joint2's rotation is not affected by the rotation of Joint1.

        // We can use the cosine law to calculate the angles of a triangle formed by the two links (LA, LB)
        //        J2
        //       /  \
        //     LA    LB
        //    /       \
        //  J1  - - -  EE
        // A = arccos((b^2 + c^2 - a^2) / 2bc)

        XYPair targetEndEffectorPosition = new XYPair(targetX, targetZ);

        // Check that the target position is within reach
        if (targetEndEffectorPosition.getMagnitude() > this.configuration.getUpperArmLength() + this.configuration.getLowerArmLength()) {
            return new ArmPositionState(new Rotation2d(0), new Rotation2d(0), false);
        }

        double angleLowerJoint = getAngleFromCosineLaw(
                this.configuration.getLowerArmLength(),
                targetEndEffectorPosition.getMagnitude(),
                this.configuration.getUpperArmLength()
        );
        double angleUpperJoint = getAngleFromCosineLaw(
                this.configuration.getLowerArmLength(),
                this.configuration.getUpperArmLength(),
                targetEndEffectorPosition.getMagnitude()
        );
        double angleHorizonToLowerJoint = targetEndEffectorPosition.getAngle();

        Rotation2d angleLowerJointRelativeToHorizon = Rotation2d.fromDegrees(angleLowerJoint)
                .plus(Rotation2d.fromDegrees(angleHorizonToLowerJoint));
        Rotation2d angleUpperJointRelativeToHorizon = angleLowerJointRelativeToHorizon
                .plus(Rotation2d.fromDegrees(angleUpperJoint))
                .plus(Rotation2d.fromDegrees(180));

        return new ArmPositionState(angleLowerJointRelativeToHorizon, angleUpperJointRelativeToHorizon, true);
    }

    private static double getAngleFromCosineLaw(double adjacent1Length, double adjacent2length, double oppositeLength) {
        return Math.toDegrees(
                Math.acos(
                    (Math.pow(adjacent1Length, 2) + Math.pow(adjacent2length, 2) - Math.pow(oppositeLength, 2))
                            / (2 * adjacent1Length * adjacent2length)));
    }
}
