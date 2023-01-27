package competition.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;

public class ArmPositionState {
    private final Rotation2d lowerJointRotation;

    private final Rotation2d upperJointRotation;

    public ArmPositionState(Rotation2d lowerJointRotation, Rotation2d upperJointRotation) {
        this.lowerJointRotation = lowerJointRotation;
        this.upperJointRotation = upperJointRotation;
    }

    /**
     * Gets the lower joint rotation
     * @return The lower joint rotation, with 0 degrees representing straight forwards towards the horizon.
     */
    public Rotation2d getLowerJointRotation() {
        return lowerJointRotation;
    }

    /**
     * Gets the upper joint rotation
     * @return The upper joint rotation, with 0 degrees representing straight forwards towards the horizon.
     */
    public Rotation2d getUpperJointRotation() {
        return upperJointRotation;
    }
}
