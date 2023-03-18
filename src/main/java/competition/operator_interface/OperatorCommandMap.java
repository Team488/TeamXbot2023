package competition.operator_interface;

import competition.auto_programs.BasicMobilityPoints;
import competition.auto_programs.BlueBottomScoringPath;
import competition.auto_programs.BlueExitCommunityAndBalanceProgram;
import competition.auto_programs.BlueScoringPositionFiveToBalanceProgram;
import competition.auto_programs.EjectLowThenBalanceProgram;
import competition.auto_programs.EjectLowThenBalanceWithMobilityProgram;
import competition.auto_programs.EjectLowThenExitHighProgram;
import competition.auto_programs.EjectLowThenExitLowProgram;
import competition.auto_programs.ParameterizedAutonomousProgram;
import competition.auto_programs.ScoreCubeHighThenBalanceProgram;
import competition.auto_programs.ScoreCubeHighThenLeaveProgram;
import competition.commandgroups.MoveCollectedGamepieceToArmCommandGroup;
import competition.commandgroups.ScoreCubeMidCommandGroup;
import competition.subsystems.arm.UnifiedArmSubsystem;
import competition.subsystems.arm.UnifiedArmSubsystem.KeyArmPosition;
import competition.subsystems.arm.UnifiedArmSubsystem.RobotFacing;
import competition.subsystems.arm.commands.ControlEndEffectorPositionCommand;
import competition.subsystems.arm.commands.SetArmsToKeyArmPositionCommand;
import competition.subsystems.arm.commands.SimpleSafeArmRouterCommand;
import competition.subsystems.arm.commands.SimpleXZRouterCommand;
import competition.subsystems.claw.ClawGripperMotorSubsystem;
import competition.subsystems.claw.ClawSubsystem;
import competition.subsystems.claw.CloseClawCommand;
import competition.subsystems.claw.OpenClawCommand;
import competition.subsystems.collector.CollectorSubsystem;
import competition.subsystems.collector.commands.CollectIfSafeCommand;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.drive.commands.AutoBalanceCommand;
import competition.subsystems.drive.commands.BrakeCommand;
import competition.subsystems.drive.commands.DebuggingSwerveWithJoysticksCommand;
import competition.subsystems.drive.commands.GoToNextActiveSwerveModuleCommand;
import competition.subsystems.drive.commands.MoveLeftInchByInchCommand;
import competition.subsystems.drive.commands.MoveRightInchByInchCommand;
import competition.subsystems.drive.commands.ManualBalanceModeCommand;
import competition.subsystems.drive.commands.PositionDriveWithJoysticksCommand;
import competition.subsystems.drive.commands.PositionMaintainerCommand;
import competition.subsystems.drive.commands.SetSwerveMotorControllerPidParametersCommand;
import competition.subsystems.drive.commands.SwerveDriveWithJoysticksCommand;
import competition.subsystems.drive.commands.SwerveToNearestScoringPositionCommand;
import competition.subsystems.drive.commands.SwerveToNextScoringPositionCommand;
import competition.subsystems.drive.commands.SwerveToPointCommand;
import competition.subsystems.drive.commands.TurnLeft90DegreesCommand;
import competition.subsystems.drive.commands.VelocityDriveWithJoysticksCommand;
import competition.subsystems.drive.commands.VelocityMaintainerCommand;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import xbot.common.command.NamedInstantCommand;
import xbot.common.controls.sensors.XXboxController.XboxButton;
import xbot.common.controls.sensors.buttons.ChordTrigger;
import xbot.common.math.XYPair;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.autonomous.SetAutonomousCommand;
import xbot.common.subsystems.pose.commands.SetRobotHeadingCommand;

import javax.inject.Inject;
import javax.inject.Provider;
import javax.inject.Singleton;
import java.security.KeyStore;

/**
 * Maps operator interface buttons to commands
 */
@Singleton
public class OperatorCommandMap {

    @Inject
    public OperatorCommandMap() {
    }

    @Inject
    public void setupDriveCommands(
            OperatorInterface oi,
            SetRobotHeadingCommand resetHeadingCube,
            Provider<SetRobotHeadingCommand> headingProvider,
            ChordTrigger.ChordTriggerFactory chordFactory,
            DriveSubsystem drive,
            PoseSubsystem pose,
            GoToNextActiveSwerveModuleCommand nextModule,
            DebuggingSwerveWithJoysticksCommand debugSwerve,
            SwerveDriveWithJoysticksCommand regularSwerve,
            PositionMaintainerCommand positionMaintainer,
            PositionDriveWithJoysticksCommand positionDrive,
            VelocityDriveWithJoysticksCommand velocityDrive,
            BrakeCommand setWheelsToXMode,
            SwerveToNearestScoringPositionCommand swerveNearestScoring,
            Provider<SwerveToNextScoringPositionCommand> swerveNextScoringProvider,
            MoveLeftInchByInchCommand moveLeft,
            MoveRightInchByInchCommand moveRight,
            ManualBalanceModeCommand setManualBalanceMode
            ) {

        resetHeadingCube.setHeadingToApply(pose.rotateAngleBasedOnAlliance(Rotation2d.fromDegrees(-180)).getDegrees());
        SetRobotHeadingCommand forwardHeading = headingProvider.get();
        SetRobotHeadingCommand backwardHeading = headingProvider.get();
        SetRobotHeadingCommand resetHeading = headingProvider.get();
        resetHeading.setHeadingToApply(() -> pose.rotateAngleBasedOnAlliance(Rotation2d.fromDegrees(0)).getDegrees());
        forwardHeading.setHeadingToApply(() -> pose.rotateAngleBasedOnAlliance(Rotation2d.fromDegrees(0)).getDegrees());
        backwardHeading.setHeadingToApply(() -> pose.rotateAngleBasedOnAlliance(Rotation2d.fromDegrees(180)).getDegrees());
        forwardHeading.includeOnSmartDashboard("setHeadingForward");
        backwardHeading.includeOnSmartDashboard("setHeadingBackward");
        NamedInstantCommand resetPosition = new NamedInstantCommand("Reset Position",
                () -> pose.setCurrentPosition(0, 0));
        ParallelCommandGroup resetPose = new ParallelCommandGroup(resetPosition, resetHeading);

        NamedInstantCommand resetPositionCube = new NamedInstantCommand("Reset Position Cube",
                () -> pose.setCurrentPosition(70, 102));
        ParallelCommandGroup resetPoseCube = new ParallelCommandGroup(resetPositionCube, resetHeadingCube);

        oi.driverGamepad.getifAvailable(XboxButton.A).onTrue(resetPose);
        oi.driverGamepad.getifAvailable(XboxButton.Back).onTrue(regularSwerve);

        NamedInstantCommand enableCollectorRotation =
                new NamedInstantCommand("Enable Collector Rotation", () -> drive.setCollectorOrientedTurningActive(true));
        NamedInstantCommand disableCollectorRotation =
                new NamedInstantCommand("Disable Collector Rotation", () -> drive.setCollectorOrientedTurningActive(false));

        var povDown = oi.driverGamepad.getPovIfAvailable(180);
        var povLeft = oi.driverGamepad.getPovIfAvailable(270);
        var povRight = oi.driverGamepad.getPovIfAvailable(90);

        var brakesButton = oi.driverGamepad.getifAvailable(XboxButton.X);
        chordFactory.create(
                brakesButton,
                povLeft).whileTrue(moveLeft);
        chordFactory.create(
                brakesButton,
                povRight).whileTrue(moveRight);
        chordFactory.create(
                chordFactory.create(
                        povLeft.negate(),
                        povRight.negate()
                ),
                brakesButton
        ).whileTrue(setWheelsToXMode);

        //oi.driverGamepad.getPovIfAvailable(0).onTrue(enableCollectorRotation);
        //oi.driverGamepad.getPovIfAvailable(180).onTrue(disableCollectorRotation);


        positionMaintainer.includeOnSmartDashboard("Drive Position Maintainer");
        velocityDrive.includeOnSmartDashboard("Drive Velocity with Joysticks");
        positionDrive.includeOnSmartDashboard("Drive Position with Joysticks");

        //oi.driverGamepad.getifAvailable(XboxButton.B).whileTrue(setWheelsToXMode);
        oi.driverGamepad.getifAvailable(XboxButton.B).onTrue(setManualBalanceMode);

        brakesButton.whileTrue(setWheelsToXMode);

        var scoringPositionModeButton = oi.driverGamepad.getifAvailable(XboxButton.Y);
        chordFactory.create(
                scoringPositionModeButton,
                povDown
        ).whileTrue(swerveNearestScoring);
        var swerveLeftScoringPosition = swerveNextScoringProvider.get();
        swerveLeftScoringPosition.setDirection(SwerveToNextScoringPositionCommand.TargetDirection.Left);
        var swerveRightScoringPosition = swerveNextScoringProvider.get();
        swerveRightScoringPosition.setDirection(SwerveToNextScoringPositionCommand.TargetDirection.Right);
        chordFactory.create(
                scoringPositionModeButton,
                povLeft
        ).whileTrue(swerveLeftScoringPosition);
        chordFactory.create(
                scoringPositionModeButton,
                povRight
        ).whileTrue(swerveRightScoringPosition);

        StartEndCommand activateJustPrecisionRotation = new StartEndCommand(
                () -> drive.setPrecisionRotationActive(true),
                () -> drive.setPrecisionRotationActive(false));
        scoringPositionModeButton.whileTrue(activateJustPrecisionRotation);
    }

    @Inject
    public void setupAutonomousDriveCommands(
            OperatorInterface oi,
            VelocityMaintainerCommand velocityMaintainer,
            AutoBalanceCommand balanceCommand) {
        //oi.driverGamepad.getXboxButton(XboxButton.Start).whileTrue(balanceCommand);
        //oi.driverGamepad.getXboxButton(XboxButton.B).onTrue(velocityMaintainer);
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
        StartEndCommand activatePrecisionDriving = new StartEndCommand(
                () -> {
                    drive.setExtremePrecisionTranslationActive(true);
                    drive.setPrecisionRotationActive(true);
                },
                () -> {
                    drive.setExtremePrecisionTranslationActive(false);
                    drive.setPrecisionRotationActive(false);
                });

        StartEndCommand activateExtremePrecisionDriving = new StartEndCommand(
                () -> {
                    drive.setExtremePrecisionTranslationActive(true);
                    drive.setPrecisionRotationActive(true);
                },
                () -> {
                    drive.setExtremePrecisionTranslationActive(false);
                    drive.setPrecisionRotationActive(false);
                });

        // Simple robot oriented drive
        StartEndCommand activateRobotOrientedDrive = new StartEndCommand(
                () -> drive.setIsRobotOrientedDrive(true),
                () -> drive.setIsRobotOrientedDrive(false));

        oi.driverGamepad.getifAvailable(XboxButton.LeftBumper).whileTrue(drive.createUnlockFullDrivePowerCommand());
        oi.driverGamepad.getifAvailable(XboxButton.RightBumper).whileTrue(activatePrecisionDriving);
    }

    @Inject
    public void setupAutonomousCommands(Provider<SetAutonomousCommand> setAutonomousCommandProvider,
                                        OperatorInterface oi,
                                        BlueScoringPositionFiveToBalanceProgram blueScoringPositionFiveToBalanceProgram,
                                        BlueExitCommunityAndBalanceProgram blueExitCommunityAndBalanceProgram,
                                        ScoreCubeHighThenLeaveProgram scoreCubeHighThenLeave,
                                        ScoreCubeHighThenBalanceProgram scoreCubeHighThenBalance,
                                        EjectLowThenBalanceProgram ejectLowThenBalance,
                                        EjectLowThenBalanceWithMobilityProgram ejectLowThenBalanceWithMobility,
                                        EjectLowThenExitLowProgram ejectLowThenExitLow,
                                        EjectLowThenExitHighProgram ejectLowThenExitHigh,
                                        ParameterizedAutonomousProgram parameterizedAutonomousProgram,
                                        ScoreCubeMidCommandGroup scoreCubeMid) {

        var scoreCubeMidThenStop = setAutonomousCommandProvider.get();
        scoreCubeMidThenStop.setAutoCommand(scoreCubeMid);
        scoreCubeMidThenStop.includeOnSmartDashboard("AutoPrograms/ScoreCubeMidThenStop");
        oi.experimentalGamepad.getifAvailable(XboxButton.LeftBumper).onTrue(scoreCubeMidThenStop);

        // These three programs have all been tested to "work" on blocks at least once.
        var setPositionFiveToBalance = setAutonomousCommandProvider.get();
        setPositionFiveToBalance.setAutoCommand(blueScoringPositionFiveToBalanceProgram);
        setPositionFiveToBalance.includeOnSmartDashboard("AutoPrograms/SetBlueScoringPositionFiveToBalanceProgram");

        var setPositionFiveMobilityThenBalance = setAutonomousCommandProvider.get();
        setPositionFiveMobilityThenBalance.setAutoCommand(blueExitCommunityAndBalanceProgram);
        setPositionFiveMobilityThenBalance.includeOnSmartDashboard("AutoPrograms/SetBlueExitCommunityAndBalanceProgram");

        var setScoreCubeHighThenLeave = setAutonomousCommandProvider.get();
        setScoreCubeHighThenLeave.setAutoCommand(scoreCubeHighThenLeave);
        setScoreCubeHighThenLeave.includeOnSmartDashboard("AutoPrograms/SetScoreCubeHighThenLeave");

        var setScoreCubeHighThenBalance = setAutonomousCommandProvider.get();
        setScoreCubeHighThenBalance.setAutoCommand(scoreCubeHighThenBalance);
        setScoreCubeHighThenBalance.includeOnSmartDashboard("AutoPrograms/SetScoreCubeHighThenBalance");

        oi.experimentalGamepad.getPovIfAvailable(0).onTrue(setPositionFiveToBalance);
        oi.experimentalGamepad.getPovIfAvailable(90).onTrue(setPositionFiveMobilityThenBalance);
        oi.experimentalGamepad.getPovIfAvailable(180).onTrue(setScoreCubeHighThenLeave);
        oi.experimentalGamepad.getPovIfAvailable(270).onTrue(setScoreCubeHighThenBalance);

        // These four programs seem reliable, and are based on the above programs but without further testing I'm concerned.
        // Should be prioritized for testing, especially the basic eject & balance combo.
        // Mapping the four of these to buttons on the gamepad.

        var setEjectLowThenBalance = setAutonomousCommandProvider.get();
        setEjectLowThenBalance.setAutoCommand(ejectLowThenBalance);
        setEjectLowThenBalance.includeOnSmartDashboard("AutoPrograms/SetEjectLowThenBalance");

        var setEjectLowThenBalanceWithMobility = setAutonomousCommandProvider.get();
        setEjectLowThenBalanceWithMobility.setAutoCommand(ejectLowThenBalanceWithMobility);
        setEjectLowThenBalanceWithMobility.includeOnSmartDashboard("AutoPrograms/SetEjectLowThenBalanceWithMobility");

        var setEjectLowThenExitLow = setAutonomousCommandProvider.get();
        setEjectLowThenExitLow.setAutoCommand(ejectLowThenExitLow);
        setEjectLowThenExitLow.includeOnSmartDashboard("AutoPrograms/SetEjectLowThenExitLow");

        var setEjectLowThenExitHigh = setAutonomousCommandProvider.get();
        setEjectLowThenExitHigh.setAutoCommand(ejectLowThenExitHigh);
        setEjectLowThenExitHigh.includeOnSmartDashboard("AutoPrograms/SetEjectLowThenExitHigh");

        oi.experimentalGamepad.getifAvailable(XboxButton.A).onTrue(setEjectLowThenBalance);
        oi.experimentalGamepad.getifAvailable(XboxButton.B).onTrue(setEjectLowThenBalanceWithMobility);
        oi.experimentalGamepad.getifAvailable(XboxButton.X).onTrue(setEjectLowThenExitLow);
        oi.experimentalGamepad.getifAvailable(XboxButton.Y).onTrue(setEjectLowThenExitHigh);

        // This is highly experimental, but gives the drive team a lot of flexibility with alliance partners.

        var setParameterizedAutonomousProgram = setAutonomousCommandProvider.get();
        setParameterizedAutonomousProgram.setAutoCommand(parameterizedAutonomousProgram);
        setParameterizedAutonomousProgram.includeOnSmartDashboard("AutoPrograms/SetParamaterizedAutonomousProgram");

        // There are two autonomous
    }

    @Inject
    public void setupArmCommands(
            OperatorInterface oi,
            UnifiedArmSubsystem arm,
            OpenClawCommand openClaw,
            CloseClawCommand closeClaw,
            ClawSubsystem claw,
            ClawGripperMotorSubsystem gripperMotorSubsystem,
            Provider<SimpleSafeArmRouterCommand> armPositionCommandProvider,
            Provider<ControlEndEffectorPositionCommand> endEffectorPositionCommandProvider,
            Provider<SimpleXZRouterCommand> simpleXZRouterCommandProvider,
            SimpleSafeArmRouterCommand router,
            ScoreCubeHighThenLeaveProgram scoreCubeHigh,
            ScoreCubeMidCommandGroup scoreCubeMid,
            CollectorSubsystem collector,
            CollectIfSafeCommand collectIfSafe,
            MoveCollectedGamepieceToArmCommandGroup moveCollectedGamepieceToArmCommandGroup,
            SetArmsToKeyArmPositionCommand setArmsToCollectPositionCommand,
            SetArmsToKeyArmPositionCommand setArmsToScoreHighCommand,
            SetArmsToKeyArmPositionCommand setArmsToScoreMediumCommand,
            SetArmsToKeyArmPositionCommand setArmsToScoreLowCommand,
            SetArmsToKeyArmPositionCommand setArmsToSubstationCollection,
            ChordTrigger.ChordTriggerFactory chordTriggerFactory) {

        var uncalibrateArms = arm.createForceUncalibratedCommand();
        var recalibrateArms = arm.createForceCalibratedCommand();

        setArmsToCollectPositionCommand.setTargetSupplier(
                () -> UnifiedArmSubsystem.KeyArmPosition.AcquireFromCollector,
                () -> UnifiedArmSubsystem.RobotFacing.Forward);

        setArmsToScoreHighCommand.setTargetSupplier(
                () -> KeyArmPosition.HighGoal,
                () -> RobotFacing.Forward
        );

        setArmsToScoreMediumCommand.setTargetSupplier(
                () -> KeyArmPosition.MidGoal,
                () -> RobotFacing.Forward
        );

        setArmsToScoreLowCommand.setTargetSupplier(
                () -> KeyArmPosition.LowGoal,
                () -> RobotFacing.Forward
        );

        setArmsToSubstationCollection.setTargetSupplier(
                () -> KeyArmPosition.LoadingTray,
                () -> RobotFacing.Forward
        );

        var engageSpecialUpperArmOverride = arm.createEngageSpecialUpperArmOverride();
        var disableSpecialUpperArmOverride = arm.createDisableSpecialUpperArmOverride();

        /*
        oi.operatorGamepad.getPovIfAvailable(0).onTrue(engageSpecialUpperArmOverride);
        oi.operatorGamepad.getPovIfAvailable(90).onTrue(engageSpecialUpperArmOverride);
        oi.operatorGamepad.getPovIfAvailable(180).onTrue(disableSpecialUpperArmOverride); // This is the disable!
        oi.operatorGamepad.getPovIfAvailable(270).onTrue(engageSpecialUpperArmOverride);
         */

        var doubleJoystickButtonpress = chordTriggerFactory.create(
                oi.operatorGamepad.getifAvailable(XboxButton.LeftStick),
                oi.operatorGamepad.getifAvailable(XboxButton.RightStick));

        doubleJoystickButtonpress.onTrue(recalibrateArms);

        SimpleSafeArmRouterCommand setLow = armPositionCommandProvider.get();
        setLow.setTarget(KeyArmPosition.LowGoal, RobotFacing.Forward);
        SimpleSafeArmRouterCommand setMid = armPositionCommandProvider.get();
        setMid.setTarget(KeyArmPosition.MidGoal, RobotFacing.Forward);
        SimpleSafeArmRouterCommand setHigh = armPositionCommandProvider.get();
        setHigh.setTarget(KeyArmPosition.HighGoal, RobotFacing.Forward);
        SimpleSafeArmRouterCommand setRetract = armPositionCommandProvider.get();
        setRetract.setTarget(KeyArmPosition.FullyRetracted, RobotFacing.Forward);
        SimpleSafeArmRouterCommand setGround = armPositionCommandProvider.get();
        setGround.setTarget(KeyArmPosition.Ground, RobotFacing.Forward);
        SimpleSafeArmRouterCommand setSubstation = armPositionCommandProvider.get();
        setSubstation.setTarget(KeyArmPosition.LoadingTray, RobotFacing.Forward);
        SimpleSafeArmRouterCommand setPickupFromCollector = armPositionCommandProvider.get();
        setPickupFromCollector.setTarget(KeyArmPosition.AcquireFromCollector, RobotFacing.Forward);

        var setlowXZ = simpleXZRouterCommandProvider.get();
        setlowXZ.setKeyPointFromKeyArmPosition(KeyArmPosition.LowGoal, RobotFacing.Forward);
        var setMidXZ = simpleXZRouterCommandProvider.get();
        setMidXZ.setKeyPointFromKeyArmPosition(KeyArmPosition.MidGoal, RobotFacing.Forward);
        var setHighXZ = simpleXZRouterCommandProvider.get();
        setHighXZ.setKeyPointFromKeyArmPosition(KeyArmPosition.HighGoal, RobotFacing.Forward);
        var setRetractXZ = simpleXZRouterCommandProvider.get();
        setRetractXZ.setKeyPointFromKeyArmPosition(KeyArmPosition.FullyRetracted, RobotFacing.Forward);
        var setSubstationXZ = simpleXZRouterCommandProvider.get();
        setSubstationXZ.setKeyPointFromKeyArmPosition(KeyArmPosition.LoadingTray, RobotFacing.Forward);

        var setPrepareToPickupFromCollectorXZ = simpleXZRouterCommandProvider.get();
        setPrepareToPickupFromCollectorXZ.setKeyPointFromKeyArmPosition(
                KeyArmPosition.PrepareToAcquireFromCollector, RobotFacing.Forward);

        var prepareToPickupAndOpenClaw = new ParallelCommandGroup(
                setPrepareToPickupFromCollectorXZ,
                new InstantCommand(claw::open, claw)
        );

        var smartOrDumbCollectionMode = new ConditionalCommand(
                setArmsToCollectPositionCommand,
                moveCollectedGamepieceToArmCommandGroup,
                arm::getEngageSpecialUpperArmOverride
        );

        var smartOrDumbScoreHigh = new ConditionalCommand(
                setArmsToScoreHighCommand,
                setHighXZ,
                arm::getEngageSpecialUpperArmOverride
        );

        var smartOrDumbScoreMedium = new ConditionalCommand(
                setArmsToScoreMediumCommand,
                setMidXZ,
                arm::getEngageSpecialUpperArmOverride
        );

        var smartOrDumbScoreLow = new ConditionalCommand(
                setArmsToScoreLowCommand,
                setlowXZ,
                arm::getEngageSpecialUpperArmOverride
        );

        var smartOrDumbCollectFromSubstation = new ConditionalCommand(
                setArmsToSubstationCollection,
                setSubstationXZ,
                arm::getEngageSpecialUpperArmOverride
        );

        oi.operatorGamepad.getifAvailable(XboxButton.X).onTrue(prepareToPickupAndOpenClaw);
        oi.operatorGamepad.getifAvailable(XboxButton.B).onTrue(smartOrDumbScoreMedium);
        oi.operatorGamepad.getifAvailable(XboxButton.Y).onTrue(smartOrDumbScoreHigh);
        oi.operatorGamepad.getifAvailable(XboxButton.A).onTrue(smartOrDumbCollectionMode);
        //oi.operatorGamepad.getifAvailable(XboxButton.RightBumper).onTrue(smartOrDumbCollectFromSubstation);

        InstantCommand setCubeMode = new InstantCommand(
                () -> {
                    Logger log = LogManager.getLogger(OperatorCommandMap.class);
                    log.info("Setting cube mode");
                    arm.setGamePieceMode(UnifiedArmSubsystem.GamePieceMode.Cube);
                });

        InstantCommand
                setConeMode = new InstantCommand(
                () -> {
                    Logger log = LogManager.getLogger(OperatorCommandMap.class);
                    log.info("Setting cone mode");
                    arm.setGamePieceMode(UnifiedArmSubsystem.GamePieceMode.Cone);
                });

        // Include on SmartDashboard only, since this is only expected to be used in pit
        SimpleSafeArmRouterCommand armToStartingPosition = armPositionCommandProvider.get();
        armToStartingPosition.setTarget(UnifiedArmSubsystem.KeyArmPosition.StartingPosition, UnifiedArmSubsystem.RobotFacing.Forward);
        armToStartingPosition.includeOnSmartDashboard("Arm to starting position");

        oi.operatorGamepad.getifAvailable(XboxButton.Back).onTrue(setConeMode);
        oi.operatorGamepad.getifAvailable(XboxButton.Start).onTrue(setCubeMode);

        router.setTarget(UnifiedArmSubsystem.KeyArmPosition.MidGoal, UnifiedArmSubsystem.RobotFacing.Forward);

        //Left Bumper opens claw if cube, closes if cone and turns on motor
       ConditionalCommand grabGamePiece = new ConditionalCommand(openClaw.alongWith(gripperMotorSubsystem.createIntakeCommand()),
                closeClaw.alongWith(gripperMotorSubsystem.createIntakeCommand()),
                () -> arm.isCubeMode());
        oi.operatorGamepad.getifAvailable(XboxButton.RightBumper).whileTrue(grabGamePiece);
        //reverse motor
        oi.operatorGamepad.getifAvailable(XboxButton.LeftBumper).whileTrue(gripperMotorSubsystem.setEject(-1.0));

        oi.operatorGamepad.getifAvailable(XboxButton.RightTrigger).whileTrue(collector.getCollectThenRetractCommand());
        oi.operatorGamepad.getifAvailable(XboxButton.LeftTrigger).whileTrue(collector.getEjectThenStopCommand());

        SmartDashboard.putData("ScoreCubeHigh", scoreCubeHigh);

        ControlEndEffectorPositionCommand moveUp = endEffectorPositionCommandProvider.get();
        moveUp.setDirection(new XYPair(0, 1));
        ControlEndEffectorPositionCommand moveForward = endEffectorPositionCommandProvider.get();
        moveForward.setDirection(new XYPair(1, 0));
        ControlEndEffectorPositionCommand moveBack = endEffectorPositionCommandProvider.get();
        moveBack.setDirection(new XYPair(-1, 0));
        ControlEndEffectorPositionCommand moveDown = endEffectorPositionCommandProvider.get();
        moveDown.setDirection(new XYPair(0, -1));

        oi.operatorGamepad.getPovIfAvailable(0).whileTrue(moveUp);
        oi.operatorGamepad.getPovIfAvailable(90).whileTrue(moveForward);
        oi.operatorGamepad.getPovIfAvailable(180).whileTrue(moveDown);
        oi.operatorGamepad.getPovIfAvailable(270).whileTrue(moveBack);
    }

}
