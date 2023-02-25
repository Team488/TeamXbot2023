package competition.operator_interface;

import competition.auto_programs.BasicMobilityPoints;
import competition.auto_programs.BlueBottomScoringPath;
import competition.subsystems.arm.UnifiedArmSubsystem;
import competition.subsystems.arm.UnifiedArmSubsystem.KeyArmPosition;
import competition.subsystems.arm.UnifiedArmSubsystem.RobotFacing;
import competition.subsystems.arm.commands.SimpleSafeArmRouterCommand;
import competition.subsystems.claw.ClawSubsystem;
import competition.subsystems.collector.CollectorSubsystem;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import org.apache.log4j.LogManager;
import org.apache.log4j.Logger;
import xbot.common.command.NamedInstantCommand;
import xbot.common.command.SmartDashboardCommandPutter;
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
            Provider<SetRobotHeadingCommand> headingProvider,
            DriveSubsystem drive,
            PoseSubsystem pose,
            DebuggingSwerveWithJoysticksCommand debugSwerve,
            GoToNextActiveSwerveModuleCommand nextModule,
            SwerveDriveWithJoysticksCommand regularSwerve,
            VisionSubsystem vision,
            PositionMaintainerCommand positionMaintainer,
            PositionDriveWithJoysticksCommand positionDrive,
            VelocityDriveWithJoysticksCommand velocityDrive) {
        SetRobotHeadingCommand resetHeading = headingProvider.get();
        SetRobotHeadingCommand forwardHeading = headingProvider.get();
        SetRobotHeadingCommand backwardHeading = headingProvider.get();
        resetHeading.setHeadingToApply(pose.rotateAngleBasedOnAlliance(Rotation2d.fromDegrees(0)).getDegrees());

        forwardHeading.setHeadingToApply(pose.rotateAngleBasedOnAlliance(Rotation2d.fromDegrees(0)).getDegrees());
        backwardHeading.setHeadingToApply(pose.rotateAngleBasedOnAlliance(Rotation2d.fromDegrees(180)).getDegrees());
        forwardHeading.includeOnSmartDashboard("setHeadingForward");
        backwardHeading.includeOnSmartDashboard("setHeadingBackward");
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


        positionMaintainer.includeOnSmartDashboard("Drive Position Maintainer");
        velocityDrive.includeOnSmartDashboard("Drive Velocity with Joysticks");
        positionDrive.includeOnSmartDashboard("Drive Position with Joysticks");
    }

    @Inject
    public void setupAutonomousDriveCommands(
            OperatorInterface oi,
            VelocityMaintainerCommand velocityMaintainer,
            AutoBalanceCommand balanceCommand) {
        oi.driverGamepad.getXboxButton(XboxButton.Start).whileTrue(balanceCommand);
        oi.driverGamepad.getXboxButton(XboxButton.B).onTrue(velocityMaintainer);
        velocityMaintainer.includeOnSmartDashboard("Drive Velocity Maintainer");
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

        StartEndCommand activatePrecisionTranslation = new StartEndCommand(
                () -> drive.setPrecisionTranslationActive(true),
                () -> drive.setPrecisionTranslationActive(false)
        );

        ParallelCommandGroup activatePrecisionRotAndTrans = new ParallelCommandGroup(activatePrecisionRotation, activatePrecisionTranslation);

        oi.driverGamepad.getifAvailable(XboxButton.LeftBumper).whileTrue(activateRobotOrientedDrive);
        oi.driverGamepad.getifAvailable(XboxButton.RightBumper).whileTrue(activatePrecisionRotAndTrans);
    }

    @Inject
    public void setupAutonomousCommands(Provider<SetAutonomousCommand> setAutonomousCommandProvider,
                                        OperatorInterface oi,
                                        BlueBottomScoringPath blueBottom,
                                        BasicMobilityPoints basicMobilityPoints) {
        var setBlueBottomScoring = setAutonomousCommandProvider.get();
        setBlueBottomScoring.setAutoCommand(blueBottom);
        setBlueBottomScoring.includeOnSmartDashboard("AutoPrograms/SetBlueBottomScoring");

        var setBasicMobilityPoints = setAutonomousCommandProvider.get();
        setBasicMobilityPoints.setAutoCommand(basicMobilityPoints);
        setBasicMobilityPoints.includeOnSmartDashboard("AutoPrograms/SetBasicMobilityPoints");
    }

    @Inject
    public void setupArmCommands(
            OperatorInterface oi,
            UnifiedArmSubsystem arm,
            ClawSubsystem claw,
            CollectorSubsystem collector,
            Provider<SimpleSafeArmRouterCommand> armPositionCommandProvider) {

        SimpleSafeArmRouterCommand setLow = armPositionCommandProvider.get();
        setLow.setTarget(KeyArmPosition.LowGoal, RobotFacing.Forward);
        SimpleSafeArmRouterCommand setMid = armPositionCommandProvider.get();
        setMid.setTarget(KeyArmPosition.MidGoal, RobotFacing.Forward);
        SimpleSafeArmRouterCommand setHigh = armPositionCommandProvider.get();
        setHigh.setTarget(KeyArmPosition.HighGoal, RobotFacing.Forward);
        SimpleSafeArmRouterCommand setRetract = armPositionCommandProvider.get();
        setRetract.setTarget(KeyArmPosition.FullyRetracted, RobotFacing.Forward);

        oi.operatorGamepad.getifAvailable(XboxButton.A).onTrue(setLow);
        oi.operatorGamepad.getifAvailable(XboxButton.B).onTrue(setMid);
        oi.operatorGamepad.getifAvailable(XboxButton.Y).onTrue(setHigh);
        oi.operatorGamepad.getifAvailable(XboxButton.X).onTrue(setRetract);

        InstantCommand setCubeMode = new InstantCommand(
                () -> {
                    Logger log = LogManager.getLogger(OperatorCommandMap.class);
                    log.info("Setting cube mode");
                    arm.setGamePieceMode(UnifiedArmSubsystem.GamePieceMode.Cube);
                });

        InstantCommand setConeMode = new InstantCommand(
                () -> {
                    Logger log = LogManager.getLogger(OperatorCommandMap.class);
                    log.info("Setting cone mode");
                    arm.setGamePieceMode(UnifiedArmSubsystem.GamePieceMode.Cone);
                });

        // Include on SmartDashboard only, since this is only expected to be used in pit
        SimpleSafeArmRouterCommand armToStartingPosition = armPositionCommandProvider.get();
        armToStartingPosition.setTarget(UnifiedArmSubsystem.KeyArmPosition.StartingPosition, UnifiedArmSubsystem.RobotFacing.Forward);
        armToStartingPosition.includeOnSmartDashboard("Arm to starting position");

        // Calibrate upper arm against the absolute encoder
        InstantCommand calibrateUpperArm = new InstantCommand(
                () -> {
                    Logger log = LogManager.getLogger(OperatorCommandMap.class);
                    log.info("CalibratingArms");
                    arm.calibrateAgainstAbsoluteEncoders();
                }
        );
        SmartDashboard.putData("Calibrate upper arm", calibrateUpperArm);

        /*
        oi.operatorGamepad.getifAvailable(XboxButton.LeftBumper).onTrue(setConeMode);
        oi.operatorGamepad.getifAvailable(XboxButton.RightBumper).onTrue(setCubeMode);
        */

        InstantCommand setSoftLimits = new InstantCommand(
                () -> {
                    Logger log = LogManager.getLogger(OperatorCommandMap.class);
                    log.info("Turning on soft limits");
                    arm.setSoftLimits(true);
                }
        );
        SmartDashboard.putData("EnableSoftLimits", setSoftLimits);

        //disable soft limits
        InstantCommand disableSoftLimits = new InstantCommand(
                () -> {
                    Logger log = LogManager.getLogger(OperatorCommandMap.class);
                    log.info("Turning off soft limits");
                    arm.setSoftLimits(false);
                }
        );
        SmartDashboard.putData("Turn off soft limits", disableSoftLimits);
        //engage brake
        InstantCommand engageBrake = new InstantCommand(()-> arm.setBrake(true));
        SmartDashboard.putData("EngageBreak",engageBrake);
        //disable brake
        InstantCommand disableBreak = new InstantCommand(()-> arm.setBrake(false));
        SmartDashboard.putData("DisableBreak",disableBreak);


        oi.operatorGamepad.getifAvailable(XboxButton.Start).onTrue(arm.createLowerArmTrimCommand(5.0));
        oi.operatorGamepad.getifAvailable(XboxButton.Back).onTrue(arm.createLowerArmTrimCommand(-5.0));

        InstantCommand openClaw = new InstantCommand(
                () -> {
                    Logger log = LogManager.getLogger(OperatorCommandMap.class);
                    log.info("Opening Claw");
                    claw.open();
                }
        );
        oi.operatorGamepad.getifAvailable(XboxButton.RightBumper).onTrue(openClaw);
        //close claw using right bumper
        InstantCommand closeClaw = new InstantCommand(
                () -> {
                    Logger log = LogManager.getLogger(OperatorCommandMap.class);
                    log.info("Closing Claw");
                    claw.close();
                }
        );
        oi.operatorGamepad.getifAvailable(XboxButton.RightBumper).onFalse(closeClaw);


        InstantCommand retract = new InstantCommand(
                () -> {
                    Logger log = LogManager.getLogger(OperatorCommandMap.class);
                    log.info("Retracting");
                    collector.retract();
                }
        );
        //Use left of dpad to retract collector
        oi.operatorGamepad.getPovIfAvailable(270).onTrue(retract);

        InstantCommand extend = new InstantCommand(
                () ->{
                    Logger log = LogManager.getLogger(OperatorCommandMap.class);
                    log.info("Extending");
                    collector.extend();
                }
        );
        //Use right of dpad to extend collector
        oi.operatorGamepad.getPovIfAvailable(90).onTrue(extend);

        //Use right trigger to collect game piece
        InstantCommand collect = new InstantCommand(
                () -> {
                    Logger log = LogManager.getLogger(OperatorCommandMap.class);
                    log.info("Collecting");
                    collector.intake();
                }
        );
        oi.operatorGamepad.getifAvailable(XboxButton.RightTrigger).onTrue(collect);

        //Use left trigger to eject game piece
        InstantCommand eject = new InstantCommand(
                () -> {
                    Logger log = LogManager.getLogger(OperatorCommandMap.class);
                    log.info("Ejecting");
                    collector.eject();
                }
        );
        oi.operatorGamepad.getifAvailable(XboxButton.LeftTrigger).onTrue(eject);



    }

}
