package competition.subsystems.drive.swerve;
import javax.inject.Inject;

import org.apache.log4j.Logger;

import competition.electrical_contract.ElectricalContract;
import competition.injection.swerve.SwerveInstance;
import competition.injection.swerve.SwerveSingleton;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import xbot.common.command.BaseSubsystem;
import xbot.common.math.WrappedRotation2d;
import xbot.common.math.XYPair;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.Property;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.pose.BasePoseSubsystem;

@SwerveSingleton
public class SwerveModuleSubsystem extends BaseSubsystem {
    private static Logger log = Logger.getLogger(SwerveModuleSubsystem.class);
    
    private final String label;

    private final SwerveDriveSubsystem driveSubsystem;
    private final SwerveSteeringSubsystem steeringSubsystem;

    private final DoubleProperty xOffsetInches;
    private final DoubleProperty yOffsetInches;

    private final Translation2d moduleTranslation;

    private SwerveModuleState targetState;

    @Inject
    public SwerveModuleSubsystem(SwerveInstance swerveInstance, SwerveDriveSubsystem driveSubsystem, SwerveSteeringSubsystem steeringSubsystem,
            ElectricalContract contract, PropertyFactory pf) {
        this.label = swerveInstance.getLabel();
        log.info("Creating SwerveModuleSubsystem " + this.label);
        pf.setPrefix(this);

        this.driveSubsystem = driveSubsystem;
        this.steeringSubsystem = steeringSubsystem;

        XYPair defaultModuleOffsets = contract.getSwerveModuleOffsets(swerveInstance);
        pf.setDefaultLevel(Property.PropertyLevel.Debug);
        this.xOffsetInches = pf.createPersistentProperty("XOffsetInches", defaultModuleOffsets.x);
        this.yOffsetInches = pf.createPersistentProperty("YOffsetInches", defaultModuleOffsets.y);

        this.moduleTranslation = new Translation2d(
            xOffsetInches.get() / BasePoseSubsystem.INCHES_IN_A_METER,
            yOffsetInches.get() / BasePoseSubsystem.INCHES_IN_A_METER);

        this.targetState = new SwerveModuleState();
    }

    /**
     * Sets the target steering angle and drive power for this module, in METRIC UNITS. 
     * @param swerveModuleState Metric swerve module state
     */
    public void setTargetState(SwerveModuleState swerveModuleState) {
        this.targetState = SwerveModuleState.optimize(swerveModuleState, getSteeringSubsystem().getCurrentRotation());

        this.getSteeringSubsystem().setTargetValue(new WrappedRotation2d(this.targetState.angle.getRadians()).getDegrees());
        // The kinetmatics library does everything in metric, so we need to transform that back to US Customary Units
        this.getDriveSubsystem().setTargetValue(this.targetState.speedMetersPerSecond * BasePoseSubsystem.INCHES_IN_A_METER);
    }

    /**
     * Gets the current state of the module, in METRIC UNITS.
     * @return Metric swerve module state
     */
    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(
            this.getDriveSubsystem().getCurrentValue() / BasePoseSubsystem.INCHES_IN_A_METER,
            this.getSteeringSubsystem().getCurrentRotation());
    }

    public SwerveModulePosition getcurrentPosition() {
        return new SwerveModulePosition(
            this.getDriveSubsystem().getCurrentPositionValue() / BasePoseSubsystem.INCHES_IN_A_METER,
            this.getSteeringSubsystem().getCurrentRotation());
    }

    public SwerveModuleState getTargetState() {
        return this.targetState;
    }

    @Override
    public String getPrefix() {
        return super.getPrefix() + this.label + "/";
    }

    public Translation2d getModuleTranslation() {
        return this.moduleTranslation;
    }

    public SwerveDriveSubsystem getDriveSubsystem() {
        return this.driveSubsystem;
    }

    public SwerveSteeringSubsystem getSteeringSubsystem() {
        return this.steeringSubsystem;
    }

    /***
     * Very basic drive method - bypasses all PID to directly control the motors.
     * Ensure that your command has required control of all relevant subsystems before doing this,
     * or you will be fighting the maintainers.
     * @param drivePower -1 to 1 value for nodule wheel power
     * @param steeringPower -1 to 1 value for module rotation power
     */
    public void setPowers(double drivePower, double steeringPower) {
        getDriveSubsystem().setPower(drivePower);
        getSteeringSubsystem().setPower(steeringPower);
    }
}