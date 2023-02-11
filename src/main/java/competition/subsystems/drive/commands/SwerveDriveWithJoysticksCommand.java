package competition.subsystems.drive.commands;

import javax.inject.Inject;

import competition.operator_interface.OperatorInterface;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.VisionSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import xbot.common.command.BaseCommand;
import xbot.common.logic.HumanVsMachineDecider;
import xbot.common.logic.HumanVsMachineDecider.HumanVsMachineDeciderFactory;
import xbot.common.logic.HumanVsMachineDecider.HumanVsMachineMode;
import xbot.common.logic.Latch;
import xbot.common.logic.Latch.EdgeType;
import xbot.common.math.MathUtils;
import xbot.common.math.XYPair;
import xbot.common.properties.BooleanProperty;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.drive.control_logic.HeadingModule;
import xbot.common.subsystems.drive.control_logic.HeadingModule.HeadingModuleFactory;

/**
 * The main swerve drive command that links up the human input (from gamepad
 * joysticks) to the drive subsystem.
 */
public class SwerveDriveWithJoysticksCommand extends BaseCommand {

    final DriveSubsystem drive;
    final PoseSubsystem pose;
    final OperatorInterface oi;
    final VisionSubsystem vision;
    final DoubleProperty input_exponent;
    final DoubleProperty drivePowerFactor;
    final DoubleProperty turnPowerFactor;
    final BooleanProperty absoluteOrientationMode;
    final HeadingModule headingModule;
    final Latch absoluteOrientationLatch;
    final DoubleProperty minimumMagnitudeForAbsoluteHeading;
    final DoubleProperty triggerOnlyPowerScaling;
    final DoubleProperty triggerOnlyExponent;
    final HumanVsMachineDecider decider;
    DriverStation.Alliance alliance;

    @Inject
    public SwerveDriveWithJoysticksCommand(DriveSubsystem drive, PoseSubsystem pose, OperatorInterface oi,
            PropertyFactory pf, HumanVsMachineDeciderFactory hvmFactory, HeadingModuleFactory headingModuleFactory, VisionSubsystem vision) {
        this.drive = drive;
        this.oi = oi;
        this.pose = pose;
        this.vision = vision;
        pf.setPrefix(this);
        this.input_exponent = pf.createPersistentProperty("Input Exponent", 1);
        this.drivePowerFactor = pf.createPersistentProperty("Power Factor", 1);
        this.turnPowerFactor = pf.createPersistentProperty("Turn Power Factor", 0.75);
        this.absoluteOrientationMode = pf.createPersistentProperty("Absolute Orientation Mode", true);
        this.minimumMagnitudeForAbsoluteHeading = pf.createPersistentProperty("Min Magnitude For Absolute Heading", 0.75);
        this.decider = hvmFactory.create(this.getPrefix());
        this.headingModule = headingModuleFactory.create(drive.getRotateToHeadingPid());
        this.triggerOnlyPowerScaling = pf.createPersistentProperty("TriggerOnlyPowerScaling", 0.75);
        this.triggerOnlyExponent = pf.createPersistentProperty("TriggerOnlyExponent", 2.0);

        // Set up a latch to trigger whenever we change the rotational mode. In either case,
        // there's some PIDs that will need to be reset, or goals that need updating.
        absoluteOrientationLatch = new Latch(absoluteOrientationMode.get(), EdgeType.Both, edge -> {
            if(edge == EdgeType.RisingEdge) {
                resetBeforeStartingAbsoluteOrientation();
            }
            else if(edge == EdgeType.FallingEdge) {
                resetBeforeStartingRelativeOrientation();
            }
        });

        this.addRequirements(drive);
    }

    public void setAbsoluteHeadingMode(boolean absoluteHeadingEnabled) {
        absoluteOrientationMode.set(absoluteHeadingEnabled);
    }

    @Override
    public void initialize() {
        log.info("Initializing");
        decider.reset();
        alliance = pose.getAlliance();
        resetBeforeStartingAbsoluteOrientation();
        resetBeforeStartingRelativeOrientation();
    }

    private void resetBeforeStartingAbsoluteOrientation() {
        // Set our desired heading to the current heading to avoid a surprising
        // change of heading during reset.
        drive.setDesiredHeading(pose.getCurrentHeading().getDegrees());
    }

    private void resetBeforeStartingRelativeOrientation() {
        headingModule.reset();
    }

    @Override
    public void execute() {
        // Feed the latch with our mode state, so it can reset PIDs or goals as appropriate.
        absoluteOrientationLatch.setValue(absoluteOrientationMode.get());
        
        // --------------------------------------------------
        // Translation
        // --------------------------------------------------

        // Get the current translation vector from the gamepad.
        XYPair rawTranslationVector = new XYPair(oi.driverGamepad.getLeftStickX(), oi.driverGamepad.getLeftStickY());
        rotateInputsBasedOnAlliance(rawTranslationVector);
        // preserve the angle
        double rawAngle = rawTranslationVector.getAngle();
        // scale the magnitude
        double updatedMagnitude = MathUtils.deadband(
                rawTranslationVector.getMagnitude(),
                oi.getDriverGamepadTypicalDeadband(),
                (a) -> MathUtils.exponentAndRetainSign(a, (int) input_exponent.get()));

        // create new vector with the scaled magnitude and angle
        XYPair translationIntent = XYPair.fromPolar(rawAngle-90, updatedMagnitude);

        // --------------------------------------------------
        // Rotation
        // --------------------------------------------------

        double suggestedRotatePower = 0;

        double humanRotatePowerFromStick = MathUtils.deadband(
                oi.driverGamepad.getRightStickX(),
                oi.getDriverGamepadTypicalDeadband(),
                (a) -> MathUtils.exponentAndRetainSign(a, (int) input_exponent.get()));

        double humanRotatePowerFromLeftTrigger = MathUtils.deadband(
                oi.driverGamepad.getLeftTrigger(),
                0.005,
                (a) -> MathUtils.exponentAndRetainSign(a, (int) triggerOnlyExponent.get()));

        double humanRotatePowerFromRightTrigger = MathUtils.deadband(
                oi.driverGamepad.getRightTrigger(),
                0.005,
                (a) -> MathUtils.exponentAndRetainSign(a, (int) triggerOnlyExponent.get()));

        double humanRotatePower = MathUtils.constrainDoubleToRobotScale(
            humanRotatePowerFromStick + humanRotatePowerFromLeftTrigger - humanRotatePowerFromRightTrigger);

        double humanRotatePowerFromTriggers = humanRotatePowerFromLeftTrigger - humanRotatePowerFromRightTrigger;
        
        // Michael wants the triggers to be more precise pretty much all the time, so adding a trigger-only
        // power reduction
        humanRotatePowerFromTriggers *= triggerOnlyPowerScaling.get();
                
        if (absoluteOrientationMode.get()) {
            // If we are using absolute orientation, we first need get the desired heading from the right joystick.
            // We need to only do this if the joystick has been moved past the minimumMagnitudeForAbsoluteHeading.
            // In the future, we might be able to replace the joystick with a dial or other device that can more easily
            // hold a heading.

            // One key note - we need to invert the right X, since it is already inverted once to make "typical" rotation commands
            // line up with our conventions. (Usually, a right turn is done by moving the joystick right. However, turning to the right
            // is a "negative" rotation, so the X axis is usually inverted to take that into account). 
            // By doing this inversion, the vector will better map onto a typical cartesian coordinate system.
            XYPair headingVector = new XYPair(-oi.driverGamepad.getRightStickX(), oi.driverGamepad.getRightStickY());
            rotateInputsBasedOnAlliance(headingVector);

            // The next step is to rotate the vector. The FRC frame assumes "forward" is 0 degrees, but the typical cartesian setup
            // of a joystick would have "forward" as 90 degrees.
            headingVector = headingVector.rotate(-90);

            double desiredHeading = 0;
            
            if (headingVector.getMagnitude() > minimumMagnitudeForAbsoluteHeading.get()) {
                // If the magnitude is greater than the minimum magnitude, we can use the joystick to set the heading.
                desiredHeading = headingVector.getAngle();
                
                if (pose.getHeadingResetRecently()) {
                    drive.setDesiredHeading(pose.getCurrentHeading().getDegrees());
                } else {
                    if (drive.isRotateToHubActive() && vision.getFixAcquired()) {
                        drive.setDesiredHeading(pose.getCurrentHeading().getDegrees() + vision.getBearingToHub());
                    } else {
                        drive.setDesiredHeading(desiredHeading);
                    }
                }
                suggestedRotatePower = headingModule.calculateHeadingPower(desiredHeading);
                decider.reset();
            } else {
                // If the joystick isn't deflected enough, we use the last known heading or human input.
                HumanVsMachineMode recommendedMode = decider.getRecommendedMode(humanRotatePowerFromTriggers);

                if (drive.isRotateToHubActive() && vision.getFixAcquired()) {
                    drive.setDesiredHeading(pose.getCurrentHeading().getDegrees() + vision.getBearingToHub());

                    // Force recommended mode to machine control to avoid coast between machine taking over after target acquired
                    recommendedMode = HumanVsMachineMode.MachineControl;
                }

                if (pose.getHeadingResetRecently()) {
                    drive.setDesiredHeading(pose.getCurrentHeading().getDegrees());
                }
                
                switch (recommendedMode) {
                    case Coast:
                        suggestedRotatePower = 0;
                        break;
                    case HumanControl:
                        suggestedRotatePower = scaleHumanRotationInput(humanRotatePowerFromTriggers);
                        break;
                    case InitializeMachineControl:
                        drive.setDesiredHeading(pose.getCurrentHeading().getDegrees());
                        suggestedRotatePower = 0;
                        break;
                    case MachineControl:
                        desiredHeading = drive.getDesiredHeading();
                        suggestedRotatePower = headingModule.calculateHeadingPower(desiredHeading);
                        break;
                    default:
                        suggestedRotatePower = 0;
                        break;
                }
            }
        } else {
            // If we are in the typical "rotate using joystick to turn" mode, use the Heading Assist module to get the suggested power.
            suggestedRotatePower = scaleHumanRotationInput(humanRotatePower);
        }

        // --------------------------------------------------
        // Safety scaling and sending to Drive Subsystem
        // --------------------------------------------------

        // It's not sensible to magnitudes greater than the unit vector. This isn't goldeneye - you can't go extra fast by moving diagonally!
        if (translationIntent.getMagnitude() > 1) {
            translationIntent.scale(1/translationIntent.getMagnitude());
        }

        // Scale the power down if we are in one or more precision modes
        // Rotation is scaled when deciding on human vs machine inputs
        translationIntent.scale(drive.isPrecisionTranslationActive() ? 0.5 : 1);

        // Scale the power down if requested (typically used when novices are controlling the robot)
        translationIntent = translationIntent.scale(drivePowerFactor.get());
        suggestedRotatePower *= turnPowerFactor.get();

        // Check if we need a different center of rotation
        XYPair centerOfRotationInches = new XYPair(0,0);
        if (drive.isCollectorRotationActive()) {
            centerOfRotationInches = new XYPair(36, 0);
        }

        // Rumble based on camera state
        if (drive.isRotateToHubActive()) {
            if(vision.getFixAcquired()) {
                // if we're not at our goal yet, rumble gamepad so driver knows we're not there yet
                if(!headingModule.isOnTarget()) {
                    oi.driverGamepad.getRumbleManager().rumbleGamepad(0.2, 0.05);
                } else {
                    oi.driverGamepad.getRumbleManager().stopGamepadRumble();
                }
            } else {
                // if driver is trying to align with vision, but no target acquired, rumble aggressively
                oi.driverGamepad.getRumbleManager().rumbleGamepad(0.8, 0.1);
            }
        } else {
            oi.driverGamepad.getRumbleManager().stopGamepadRumble();
        }
        
        if (drive.isRobotOrientedDriveActive()) {
            drive.move(translationIntent, suggestedRotatePower);
        } else {
            drive.fieldOrientedDrive(translationIntent, suggestedRotatePower, pose.getCurrentHeading().getDegrees(), centerOfRotationInches);
        }
    }

    private double scaleHumanRotationInput(double humanInputPower) {
        if (drive.isPrecisionRotationActive() || drive.isRobotOrientedDriveActive()) {
            return humanInputPower * 0.25;
        }
        return humanInputPower;
    }

    /**
     * Rotate the inputs by 180 degrees if the driver is on the red alliance.
     * @param translationInput The raw input value
     * @return The rotated input
     */
    private XYPair rotateInputsBasedOnAlliance(XYPair translationInput) {
        if (alliance == DriverStation.Alliance.Red) {
            translationInput.scale(-1, -1);
        }
        return translationInput;
    }
}