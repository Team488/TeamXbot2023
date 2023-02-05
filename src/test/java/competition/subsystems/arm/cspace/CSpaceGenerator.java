package competition.subsystems.arm.cspace;

import xbot.common.math.XYPair;

public class CSpaceGenerator {

    public enum IllegalReason {
        OK,
        CollidesWithRobot,
        CollidesWithCollector,
        EffectorTooHigh,
        EffectorBelowGround,
        BehindAndInFront,
        TooFarForward,
        TooFarBackward
    }

    double baseJointX = 0;
    double baseJointY = 12;

    Obstacle robotObstacle = new Obstacle(0, 6, 29, 12, "Robot");
    Obstacle intakeObstacle = new Obstacle(20, 10, 8, 12, "Intake");

    // main lower joint is 7.5" from the ground
    double lowerArmLength = 38;
    double upperArmLength = 32;

    double lowerArmJointX = 0;
    double lowerArmJointY = 13;

    XYPair lowerJoint = new XYPair(lowerArmJointX, lowerArmJointY);

    public CSpaceGenerator() {
    }

    public IllegalReason testAngles(double lowerAngle, double upperAngle) {

        XYPair upperJoint = new XYPair(
            lowerArmJointX + lowerArmLength * Math.cos(Math.toRadians(lowerAngle)),
            lowerArmJointY + lowerArmLength * Math.sin(Math.toRadians(lowerAngle))
        );

        XYPair endEffector = new XYPair(
            upperJoint.x + upperArmLength * Math.cos(Math.toRadians(upperAngle)),
            upperJoint.y + upperArmLength * Math.sin(Math.toRadians(upperAngle))
        );

        IllegalReason reason = IllegalReason.OK;

        // Check arm segments for collisions with our obstacles
        var obstacleCollisionResult = checkForCollisions(lowerJoint, upperJoint, endEffector);
        if (obstacleCollisionResult != IllegalReason.OK) {
            reason = obstacleCollisionResult;
        }

        if (endEffector.y > 6*12+6) {
            reason = IllegalReason.EffectorTooHigh;
        }

        if (endEffector.y < 0) {
            reason = IllegalReason.EffectorBelowGround;
        }

        // Check if upper joint too far back and effector too far forward
        if (upperJoint.x < -robotObstacle.width/2 && endEffector.x > robotObstacle.width/2) {
            reason = IllegalReason.BehindAndInFront;
        }
        // Also check for the reverse
        if (upperJoint.x > robotObstacle.width/2 && endEffector.x < -robotObstacle.width/2) {
            reason = IllegalReason.BehindAndInFront;
        }

        // Check if end effector too far forward
        if (endEffector.x > robotObstacle.width/2 + 48) {
            reason = IllegalReason.TooFarForward;
        }
        // Check the reverse
        if (endEffector.x < -robotObstacle.width/2 - 48) {
            reason = IllegalReason.TooFarBackward;
        }

        System.out.println(lowerAngle + "," + upperAngle + "," + reason);
        return reason;
    }

    private IllegalReason checkForCollisions(XYPair lowerJoint, XYPair upperJoint, XYPair endEffector) {
        // Check first segment against robotObstacle
        if (robotObstacle.intersectsLine(lowerJoint.x, lowerJoint.y, upperJoint.x, upperJoint.y)) {
            return IllegalReason.CollidesWithRobot;
        }
        // Check second segment against robotObstacle
        if (robotObstacle.intersectsLine(upperJoint.x, upperJoint.y, endEffector.x, endEffector.y)) {
            return IllegalReason.CollidesWithRobot;
        }
        // Check first segment against intakeObstacle
        if (intakeObstacle.intersectsLine(lowerJoint.x, lowerJoint.y, upperJoint.x, upperJoint.y)) {
            return IllegalReason.CollidesWithCollector;
        }
        // Check second segment against intakeObstacle
        if (intakeObstacle.intersectsLine(upperJoint.x, upperJoint.y, endEffector.x, endEffector.y)) {
            return IllegalReason.CollidesWithCollector;
        }
        return IllegalReason.OK;
    }
}
