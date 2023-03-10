package competition.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import xbot.common.math.MathUtils;
import xbot.common.math.WrappedRotation2d;
import xbot.common.math.XYPair;

public class ArmPositionSolver {
    private final ArmPositionSolverConfiguration configuration;

    public ArmPositionSolver(ArmPositionSolverConfiguration configuration) {
        this.configuration = configuration;
    }

    public ArmPositionSolverConfiguration getConfiguration() {
        return this.configuration;
    }

    public ArmPositionState solveArmJointPositions(XYPair targetEndEffectorPosition, XYPair currentAngles) {
        return solveArmJointPositions(targetEndEffectorPosition, currentAngles, false);
    }

    /**
     * Calculate the target rotation of each arm joint given a target end-effector position
     * @param targetEndEffectorPosition X: The distance in front of or behind the robot (in inches) relative to the fixed arm pivot.
     *                                  Y: The distance above or below the fixed pivot (in inches).
     * @param currentAngles X: The current lower joint angle.
     *                      Y: The current upper joint angle.
     */
    public ArmPositionState solveArmJointPositions(
            XYPair targetEndEffectorPosition,
            XYPair currentAngles,
            boolean forceForward) {
        // The arm is made up of two links. LinkA is fixed to a pivot point on the base of the robot (Joint1).
        // LinkB is attached to the other end of LinkA at Joint2.

        // We can use the cosine law to calculate the angles of a triangle formed by the two links (LA, LB)
        //        J2
        //       /  \
        //     LA    LB
        //    /       \
        //  J1  - - -  EE
        // A = arccos((b^2 + c^2 - a^2) / 2bc)

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

        // Two possible solutions, depending on if we started with upper arm angle greater than or less than zero.
        // We don't want to cross this, because it will produce a drastically different solution.
        if (currentAngles.y >= 0 || forceForward) {
            double angleLowerJointRelativeToHorizon = angleLowerJoint + targetEndEffectorPosition.getAngle();

            XYPair lowerArmEndpoint = new XYPair(Rotation2d.fromDegrees(angleLowerJointRelativeToHorizon)).scale(configuration.getLowerArmLength());
            XYPair upperArmEndpoint = new XYPair(Rotation2d.fromDegrees(angleLowerJoint + angleUpperJoint)).scale(configuration.getLowerArmLength());

            return new ArmPositionState(
                    Rotation2d.fromDegrees(angleLowerJointRelativeToHorizon),
                    Rotation2d.fromDegrees(angleUpperJoint), true);
        } else {
            double angleLowerJointRelativeToHorizon = targetEndEffectorPosition.getAngle() - angleLowerJoint;

            XYPair lowerArmEndpoint = new XYPair(Rotation2d.fromDegrees(angleLowerJointRelativeToHorizon)).scale(configuration.getLowerArmLength());
            XYPair upperArmEndpoint = new XYPair(Rotation2d.fromDegrees(angleLowerJoint + angleUpperJoint)).scale(configuration.getLowerArmLength());

            return new ArmPositionState(
                    Rotation2d.fromDegrees(angleLowerJointRelativeToHorizon),
                    Rotation2d.fromDegrees(-angleUpperJoint), true);
        }
    }

    private static double getAngleFromCosineLaw(double adjacent1Length, double adjacent2length, double oppositeLength) {
        double value = (Math.pow(adjacent1Length, 2) + Math.pow(adjacent2length, 2) - Math.pow(oppositeLength, 2))
                / (2 * adjacent1Length * adjacent2length);
        return Math.toDegrees(Math.acos(value));
    }

    public XYPair getPositionFromRadians(double lowerArmAngleRadians, double upperArmAngleRadians) {
        // Once the four bar on the arm was removed, the upper arm angle is now effectively coupled to the lower arm
        // angle, so the effective angle needs to be adjusted. For example, if the lower arm is at 45 degrees, and the
        // upper arm is at 45 degrees, the lower arm's true angle is 225 (or -90), which is pointed into the ground.

        double adjustedUpperArmAngleRadians = upperArmAngleRadians + (lowerArmAngleRadians + MathUtils.Tau/2.0);

        XYPair lowerArm = new XYPair(new Rotation2d(lowerArmAngleRadians)).scale(configuration.getLowerArmLength());
        XYPair upperArm = new XYPair(new WrappedRotation2d(adjustedUpperArmAngleRadians)).scale(configuration.getUpperArmLength());

        return lowerArm.add(upperArm);
    }

    public XYPair getPositionFromDegrees(double lowerArmAngleDegrees, double upperArmAngleDegrees) {
        return getPositionFromRadians(
                lowerArmAngleDegrees / 180.0 * Math.PI,
                upperArmAngleDegrees / 180.0 * Math.PI);
    }

    /**
     * If we wanted to do something like hold the upper arm horizontal while the lower arm was moving around,
     * we would need to know what angle to set the upper arm to in order to achieve an arbitrary "robot frame" angle.
     * @param lowerArmRadians The current angle of the lower arm
     * @param desiredRobotFrameRadians The desired angle of the upper arm (with respect to the robot frame)
     * @return The upper arm angle (with respect to the lower arm) needed to achieve the desired robot frame angle
     */
    public static double getUpperArmAngleNeededToAchieveRobotFrameAngle(double lowerArmRadians, double desiredRobotFrameRadians) {
        return desiredRobotFrameRadians - (lowerArmRadians + MathUtils.Tau/2.0);
    }

    public static double convertOldArmAngleToNewArmAngle(double oldLowerArmAngle, double oldUpperArmAngle) {
        return getUpperArmAngleNeededToAchieveRobotFrameAngle(
                oldLowerArmAngle / 180.0 * Math.PI,
                oldUpperArmAngle / 180.0 * Math.PI
        );
    }
}
