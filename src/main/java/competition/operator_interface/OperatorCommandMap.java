package competition.operator_interface;

import competition.auto_programs.BlueBottomScoringPath;
import competition.subsystems.arm.UnifiedArmSubsystem;
import competition.subsystems.arm.commands.SetArmsToPositionCommand;
import competition.subsystems.claw.ClawSubsystem;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.drive.commands.AutoBalanceCommand;
import competition.subsystems.drive.commands.DebuggingSwerveWithJoysticksCommand;
import competition.subsystems.drive.commands.GoToNextActiveSwerveModuleCommand;
import competition.subsystems.drive.commands.PositionDriveWithJoysticksCommand;
import competition.subsystems.drive.commands.PositionMaintainerCommand;
import competition.subsystems.drive.commands.SetSwerveMotorControllerPidParametersCommand;
import competition.subsystems.drive.commands.SwerveDriveWithJoysticksCommand;
import competition.subsystems.drive.commands.SwerveToPointCommand;
import competition.subsystems.drive.commands.TurnLeft90DegreesCommand;
import competition.subsystems.drive.commands.VelocityDriveWithJoysticksCommand;
import competition.subsystems.drive.commands.VelocityMaintainerCommand;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.VisionSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import org.apache.log4j.LogManager;
import org.apache.log4j.Logger;
import xbot.common.command.NamedInstantCommand;
import xbot.common.controls.sensors.XXboxController.XboxButton;
import xbot.common.math.XYPair;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.autonomous.SetAutonomousCommand;
import xbot.common.subsystems.pose.commands.SetRobotHeadingCommand;

import javax.inject.Inject;
import javax.inject.Provider;
import javax.inject.Singleton;

/**
 * Maps operator interface buttons to commands
 */
@Singleton
public class OperatorCommandMap {

    @Inject
    public OperatorCommandMap() {
    }

    @Inject
    public void setupDriveCommands(OperatorInterface oi,
            SetRobotHeadingCommand resetHeading,
            DriveSubsystem drive,
            PoseSubsystem pose,
            DebuggingSwerveWithJoysticksCommand debugSwerve,
            GoToNextActiveSwerveModuleCommand nextModule,
            SwerveDriveWithJoysticksCommand regularSwerve,
            VisionSubsystem vision,
            VelocityMaintainerCommand velocityMaintainer,
            PositionMaintainerCommand positionMaintainer,
            PositionDriveWithJoysticksCommand positionDrive,
            VelocityDriveWithJoysticksCommand velocityDrive) {
        resetHeading.setHeadingToApply(pose.rotateAngleBasedOnAlliance(Rotation2d.fromDegrees(0)).getDegrees());

        NamedInstantCommand resetPosition = new NamedInstantCommand("Reset Position",
                () -> pose.setCurrentPosition(0, 0));
        ParallelCommandGroup resetPose = new ParallelCommandGroup(resetPosition, resetHeading);

        StartEndCommand enableVisionRotation = new StartEndCommand(
                () -> drive.setRotateToHubActive(true),
                () -> drive.setRotateToHubActive(false));

        oi.driverGamepad.getifAvailable(XboxButton.A).onTrue(resetPose);
        //oi.driverGamepad.getifAvailable(XboxButton.RightBumper).whileTrue(enableVisionRotation);

        oi.driverGamepad.getifAvailable(XboxButton.Y).onTrue(debugSwerve);
        oi.driverGamepad.getifAvailable(XboxButton.X).onTrue(nextModule);
        oi.driverGamepad.getifAvailable(XboxButton.Back).onTrue(regularSwerve);

        NamedInstantCommand enableCollectorRotation =
                new NamedInstantCommand("Enable Collector Rotation", () -> drive.setCollectorOrientedTurningActive(true));
        NamedInstantCommand disableCollectorRotation =
                new NamedInstantCommand("Disable Collector Rotation", () -> drive.setCollectorOrientedTurningActive(false));

        oi.driverGamepad.getPovIfAvailable(0).onTrue(enableCollectorRotation);
        oi.driverGamepad.getPovIfAvailable(180).onTrue(disableCollectorRotation);

        
        velocityMaintainer.includeOnSmartDashboard("Drive Velocity Maintainer");
        positionMaintainer.includeOnSmartDashboard("Drive Position Maintainer");
        velocityDrive.includeOnSmartDashboard("Drive Velocity with Joysticks");
        positionDrive.includeOnSmartDashboard("Drive Position with Joysticks");
    }

    @Inject
    public void setupAutonomousDriveCommands(OperatorInterface oi, AutoBalanceCommand balanceCommand) {
        oi.driverGamepad.getXboxButton(XboxButton.Start).whileTrue(balanceCommand);
    }

    @Inject
    public void setupGeneralSwerveCommands(SetSwerveMotorControllerPidParametersCommand setSteeringPidValues) {
        setSteeringPidValues.includeOnSmartDashboard("Commit steering pid values");
    }

    @Inject
    public void setupMobilityCommands(OperatorInterface oi,
                                      TurnLeft90DegreesCommand turnleft90,
                                      SwerveToPointCommand swerveToPoint,

                                      DriveSubsystem drive,
                                      PropertyFactory pf) {

        pf.setPrefix("OperatorCommandMap/");
        DoubleProperty xTarget = pf.createEphemeralProperty("OI/SwerveToPointTargetX", 0);
        DoubleProperty yTarget = pf.createEphemeralProperty("OI/SwerveToPointTargetY", 0);
        DoubleProperty angleTarget = pf.createEphemeralProperty("OI/SwerveToPointTargetAngle", 0);

        swerveToPoint.setTargetSupplier(
                () -> {
                    return new XYPair(xTarget.get(), yTarget.get());
                },
                () -> {
                    return angleTarget.get();
                });

        swerveToPoint.includeOnSmartDashboard("Swerve To Point Debug");
        swerveToPoint.setMaxPower(0.35);

        // Precision Commands
        StartEndCommand activatePrecisionRotation = new StartEndCommand(
                () -> drive.setPrecisionRotationActive(true),
                () -> drive.setPrecisionRotationActive(false));

        StartEndCommand activateRobotOrientedDrive = new StartEndCommand(
                () -> drive.setIsRobotOrientedDrive(true),
                () -> drive.setIsRobotOrientedDrive(false));

        oi.driverGamepad.getifAvailable(XboxButton.LeftBumper).whileTrue(activateRobotOrientedDrive);
        oi.driverGamepad.getifAvailable(XboxButton.RightBumper).whileTrue(activatePrecisionRotation);
    }

    @Inject
    public void setupAutonomousCommands(Provider<SetAutonomousCommand> setAutonomousCommandProvider,
                                        OperatorInterface oi,
                                        BlueBottomScoringPath bluebottom) {
        var setBlueBottomScoring = setAutonomousCommandProvider.get();
        setBlueBottomScoring.includeOnSmartDashboard("AutoPrograms/SetBlueButtomScoring");
    }

    @Inject
    public void setupArmCommands(
            OperatorInterface oi,
            UnifiedArmSubsystem arm,
            ClawSubsystem claw,
            SetArmsToPositionCommand setHigh,
            SetArmsToPositionCommand setMid,
            SetArmsToPositionCommand setLow,
            SetArmsToPositionCommand setRetract) {
        /*
        oi.operatorGamepad.getifAvailable(XboxButton.B).onTrue(setLow);
        oi.operatorGamepad.getifAvailable(XboxButton.Y).onTrue(setMid);
        oi.operatorGamepad.getifAvailable(XboxButton.A).onTrue(setHigh);
        oi.operatorGamepad.getifAvailable(XboxButton.X).onTrue(setRetract);
        */
        InstantCommand firstTestPosition = new InstantCommand(
                () -> {
                    Logger log = LogManager.getLogger(OperatorCommandMap.class);
                    log.info("Setting first test position");
                    arm.setTargetValue(new XYPair(47.2, 16.85));
                });

        InstantCommand secondTestPosition = new InstantCommand(
                () -> {
                    Logger log = LogManager.getLogger(OperatorCommandMap.class);
                    log.info("Setting second test position");
                    arm.setTargetValue(new XYPair(75.6, -20.1));
                });

        InstantCommand thirdTestPosition = new InstantCommand(
                () -> {
                    Logger log = LogManager.getLogger(OperatorCommandMap.class);
                    log.info("Setting third test position");
                    arm.setTargetValue(new XYPair(67.1, -69.5));
                });

        oi.operatorGamepad.getifAvailable(XboxButton.A).onTrue(firstTestPosition);
        oi.operatorGamepad.getifAvailable(XboxButton.B).onTrue(secondTestPosition);
        oi.operatorGamepad.getifAvailable(XboxButton.X).onTrue(thirdTestPosition);


        //turn on soft limits
        InstantCommand setSoftLimits = new InstantCommand(
                () -> {
                    Logger log = LogManager.getLogger(OperatorCommandMap.class);
                    log.info("Activating soft limits");
                    arm.setSoftLimits(true);
                });
        oi.operatorGamepad.getifAvailable(XboxButton.LeftBumper).onTrue(setSoftLimits);


        //turn off soft limits
        InstantCommand disableSoftLimits = new InstantCommand(
                () -> {
                    Logger log = LogManager.getLogger(OperatorCommandMap.class);
                    log.info("Disabling soft limits");
                    arm.setSoftLimits(false);
                });

        oi.operatorGamepad.getifAvailable(XboxButton.RightBumper).onTrue(disableSoftLimits);


        InstantCommand engageBrakes = new InstantCommand(() -> arm.setBrake(true));
        InstantCommand disableBrakes = new InstantCommand(() -> arm.setBrake(false));

        //turn breaks on
        oi.operatorGamepad.getifAvailable(XboxButton.LeftTrigger).onTrue(engageBrakes);
        //turn breaks off
        oi.operatorGamepad.getifAvailable(XboxButton.RightTrigger).onTrue(disableBrakes);
        // Calibrate upper arm against the absolute encoder
        InstantCommand calibrateUpperArm = new InstantCommand(
                () -> {
                    Logger log = LogManager.getLogger(OperatorCommandMap.class);
                    log.info("CalibratingArms");
                    arm.calibrateAgainstAbsoluteEncoders();
                }
        );
        oi.operatorGamepad.getifAvailable(XboxButton.Back).onTrue(calibrateUpperArm);

        InstantCommand openClaw = new InstantCommand(
                () -> {
                    Logger log = LogManager.getLogger(OperatorCommandMap.class);
                    log.info("Opening Claw");
                    claw.open();
                }
        );
        oi.operatorGamepad.getPovIfAvailable(0).onTrue(openClaw);

        InstantCommand closeClaw = new InstantCommand(
                () -> {
                    Logger log = LogManager.getLogger(OperatorCommandMap.class);
                    log.info("Closing Claw");
                    claw.close();
                }
        );
        oi.operatorGamepad.getPovIfAvailable(180).onTrue(closeClaw);
    }

}
