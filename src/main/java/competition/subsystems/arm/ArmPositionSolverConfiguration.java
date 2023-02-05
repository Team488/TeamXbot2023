package competition.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;

public class ArmPositionSolverConfiguration {
    private final double lowerArmLength;

    private final double upperArmLength;

    private final Rotation2d minimumLowerJointAngle;

    private final Rotation2d maximumLowerJointAngle;

    private final Rotation2d minimumUpperJointAngle;

    private final Rotation2d maximumUpperJointAngle;

    public ArmPositionSolverConfiguration(
            double lowerArmLength,
            double upperArmLength,
            Rotation2d minimumLowerJointAngle,
            Rotation2d maximumLowerJointAngle,
            Rotation2d minimumUpperJointAngle,
            Rotation2d maximumUpperJointAngle
    ) {
        this.lowerArmLength = lowerArmLength;
        this.upperArmLength = upperArmLength;
        this.minimumLowerJointAngle = minimumLowerJointAngle;
        this.maximumLowerJointAngle = maximumLowerJointAngle;
        this.minimumUpperJointAngle = minimumUpperJointAngle;
        this.maximumUpperJointAngle = maximumUpperJointAngle;
    }

    public double getLowerArmLength() {
        return lowerArmLength;
    }

    public double getUpperArmLength() {
        return upperArmLength;
    }

    public Rotation2d getMinimumLowerJointAngle() {
        return minimumLowerJointAngle;
    }

    public Rotation2d getMaximumLowerJointAngle() {
        return maximumLowerJointAngle;
    }

    public Rotation2d getMinimumUpperJointAngle() {
        return minimumUpperJointAngle;
    }

    public Rotation2d getMaximumUpperJointAngle() {
        return maximumUpperJointAngle;
    }
}
