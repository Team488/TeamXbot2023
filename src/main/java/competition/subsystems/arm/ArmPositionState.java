package competition.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;

public class ArmPositionState {
    private final Rotation2d lowerJointRotation;

    private final Rotation2d upperJointRotation;

    private final boolean isSolveable;

    public ArmPositionState(Rotation2d lowerJointRotation, Rotation2d upperJointRotation, boolean isSolveable) {
        this.lowerJointRotation = lowerJointRotation;
        this.upperJointRotation = upperJointRotation;
        this.isSolveable = isSolveable;
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

    /**
     * Gets whether the solver thinks it returns a valid result
     * @return True if the solver result should be valid.
     */
    public boolean isSolveable() { return isSolveable; }
}
