package competition.operator_interface;

import competition.auto_programs.BasicMobilityPoints;
import competition.auto_programs.BlueBottomScoringPath;
import competition.auto_programs.BlueExitCommunityAndBalanceProgram;
import competition.auto_programs.BlueScoringPositionFiveToBalanceProgram;
import competition.auto_programs.ScoreCubeHighThenLeaveProgram;
import competition.subsystems.arm.UnifiedArmSubsystem;
import competition.subsystems.arm.UnifiedArmSubsystem.KeyArmPosition;
import competition.subsystems.arm.UnifiedArmSubsystem.RobotFacing;
import competition.subsystems.arm.commands.ControlEndEffectorPositionCommand;
import competition.subsystems.arm.commands.SimpleSafeArmRouterCommand;
import competition.subsystems.arm.commands.SimpleXZRouterCommand;
import competition.subsystems.claw.OpenClawCommand;
import competition.subsystems.collector.CollectorSubsystem;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.drive.commands.AutoBalanceCommand;
import competition.subsystems.drive.commands.BrakeCommand;
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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    public void setupDriveCommands(
            OperatorInterface oi,
            SetRobotHeadingCommand resetHeadingCube,
            Provider<SetRobotHeadingCommand> headingProvider,
            DriveSubsystem drive,
            PoseSubsystem pose,
            GoToNextActiveSwerveModuleCommand nextModule,
            SwerveDriveWithJoysticksCommand regularSwerve,
            PositionMaintainerCommand positionMaintainer,
            PositionDriveWithJoysticksCommand positionDrive,
            VelocityDriveWithJoysticksCommand velocityDrive,
            BrakeCommand setWheelsToXMode) {

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
        oi.driverGamepad.getifAvailable(XboxButton.Y).onTrue(resetPoseCube);

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

        oi.driverGamepad.getifAvailable(XboxButton.B).whileTrue(setWheelsToXMode);
    }

    @Inject
    public void setupAutonomousDriveCommands(
            OperatorInterface oi,
            VelocityMaintainerCommand velocityMaintainer,
            AutoBalanceCommand balanceCommand) {
        oi.driverGamepad.getXboxButton(XboxButton.Start).whileTrue(balanceCommand);
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
                                        BasicMobilityPoints basicMobilityPoints,
                                        BlueScoringPositionFiveToBalanceProgram blueScoringPositionFiveToBalanceProgram,
                                        BlueExitCommunityAndBalanceProgram blueExitCommunityAndBalanceProgram,
                                        ScoreCubeHighThenLeaveProgram scoreCubeHighThenLeave) {

        var setBlueBottomScoring = setAutonomousCommandProvider.get();
        setBlueBottomScoring.setAutoCommand(blueBottom);
        setBlueBottomScoring.includeOnSmartDashboard("AutoPrograms/SetBlueBottomScoring");

        var setBasicMobilityPoints = setAutonomousCommandProvider.get();
        setBasicMobilityPoints.setAutoCommand(basicMobilityPoints);
        setBasicMobilityPoints.includeOnSmartDashboard("AutoPrograms/SetBasicMobilityPoints");

        var setBlueCommunitySideToChargeStation = setAutonomousCommandProvider.get();
        setBlueCommunitySideToChargeStation.setAutoCommand(blueScoringPositionFiveToBalanceProgram);
        setBlueCommunitySideToChargeStation.includeOnSmartDashboard("AutoPrograms/SetBlueScoringPositionFiveToBalanceProgram");

        var setBlueMoveOutToFieldAndOntoChargePad = setAutonomousCommandProvider.get();
        setBlueMoveOutToFieldAndOntoChargePad.setAutoCommand(blueExitCommunityAndBalanceProgram);
        setBlueMoveOutToFieldAndOntoChargePad.includeOnSmartDashboard("AutoPrograms/SetBlueExitCommunityAndBalanceProgram");

        var setScoreCubeHighThenLeave = setAutonomousCommandProvider.get();
        setScoreCubeHighThenLeave.setAutoCommand(scoreCubeHighThenLeave);
        setScoreCubeHighThenLeave.includeOnSmartDashboard("AutoPrograms/SetScoreCubeHighThenLeave");
    }

    @Inject
    public void setupArmCommands(
            OperatorInterface oi,
            UnifiedArmSubsystem arm,
            OpenClawCommand openClaw,
            Provider<SimpleSafeArmRouterCommand> armPositionCommandProvider,
            Provider<ControlEndEffectorPositionCommand> endEffectorPositionCommandProvider,
            Provider<SimpleXZRouterCommand> simpleXZRouterCommandProvider,
            SimpleSafeArmRouterCommand router,
            ScoreCubeHighThenLeaveProgram scoreCubeHigh,
            CollectorSubsystem collector) {

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

        var setMidXZ = simpleXZRouterCommandProvider.get();
        setMidXZ.setKeyPointFromDirectAngles(UnifiedArmSubsystem.midGoalCubeAngles);

        oi.operatorGamepad.getifAvailable(XboxButton.A).onTrue(setLow);
        oi.operatorGamepad.getifAvailable(XboxButton.B).onTrue(setMidXZ);
        oi.operatorGamepad.getifAvailable(XboxButton.Y).onTrue(setHigh);
        oi.operatorGamepad.getifAvailable(XboxButton.X).onTrue(setRetract);
        oi.operatorGamepad.getifAvailable(XboxButton.LeftBumper).onTrue(setSubstation);

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

        oi.operatorGamepad.getifAvailable(XboxButton.Back).onTrue(setConeMode);
        oi.operatorGamepad.getifAvailable(XboxButton.Start).onTrue(setCubeMode);

        router.setTarget(UnifiedArmSubsystem.KeyArmPosition.MidGoal, UnifiedArmSubsystem.RobotFacing.Forward);
        oi.operatorGamepad.getifAvailable(XboxButton.RightBumper).whileTrue(openClaw);



        InstantCommand retract = new InstantCommand(
                () -> {
                    Logger log = LogManager.getLogger(OperatorCommandMap.class);
                    log.info("Retracting");
                    collector.retract();
                }
        );
        //Use left of dpad to retract collector
        //oi.operatorGamepad.getPovIfAvailable(270).onTrue(retract);

        InstantCommand extend = new InstantCommand(
                () ->{
                    Logger log = LogManager.getLogger(OperatorCommandMap.class);
                    log.info("Extending");
                    collector.extend();
                }
        );
        //Use right of dpad to extend collector
        //oi.operatorGamepad.getPovIfAvailable(90).onTrue(extend);

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
