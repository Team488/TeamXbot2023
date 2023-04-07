package competition.operator_interface;

import competition.auto_programs.BlueExitCommunityAndBalanceProgram;
import competition.auto_programs.BlueScoringPositionFiveToBalanceProgram;
import competition.auto_programs.EjectLowThenBalanceProgram;
import competition.auto_programs.EjectLowThenBalanceWithMobilityProgram;
import competition.auto_programs.EjectLowThenExitHighProgram;
import competition.auto_programs.EjectLowThenExitLowProgram;
import competition.auto_programs.ParameterizedAutonomousProgram;
import competition.auto_programs.ScoreCubeHighThenBalanceProgram;
import competition.auto_programs.ScoreCubeHighThenLeaveProgram;
import competition.auto_programs.support.AutonomousOracle;
import competition.commandgroups.MoveCollectedGamepieceToArmCommandGroup;
import competition.commandgroups.ScoreConeHighCommandGroup;
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
import org.apache.log4j.LogManager;
import org.apache.log4j.Logger;
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
            Provider<SwerveToNearestScoringPositionCommand> swerveNearestScoringProvider,
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

        NamedInstantCommand resetPosition = new NamedInstantCommand("Reset Position",
                () -> pose.setCurrentPosition(0, 0));
        ParallelCommandGroup resetPose = new ParallelCommandGroup(resetPosition, resetHeading);

        NamedInstantCommand resetPositionCube = new NamedInstantCommand("Reset Position Cube",
                () -> pose.setCurrentPosition(70, 102));
        ParallelCommandGroup resetPoseCube = new ParallelCommandGroup(resetPositionCube, resetHeadingCube);

        oi.driverGamepad.getifAvailable(XboxButton.A).onTrue(resetHeading);
        oi.driverGamepad.getifAvailable(XboxButton.Back).onTrue(regularSwerve);
        oi.driverGamepad.getifAvailable(XboxButton.Start).onTrue(backwardHeading);

        NamedInstantCommand enableCollectorRotation =
                new NamedInstantCommand("Enable Collector Rotation", () -> drive.setCollectorOrientedTurningActive(true));
        NamedInstantCommand disableCollectorRotation =
                new NamedInstantCommand("Disable Collector Rotation", () -> drive.setCollectorOrientedTurningActive(false));

        StartEndCommand enableGamePieceRotation = new StartEndCommand(
                        () -> drive.setGamePieceOrientatedRotationActive(true),
                        () -> drive.setGamePieceOrientatedRotationActive(false));

        oi.driverGamepad.getifAvailable(XboxButton.B).whileTrue(enableGamePieceRotation);

        var povDown = oi.driverGamepad.getPovIfAvailable(180);
        var povLeft = oi.driverGamepad.getPovIfAvailable(270);
        var povRight = oi.driverGamepad.getPovIfAvailable(90);
        var povUp = oi.driverGamepad.getPovIfAvailable(0);

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

        brakesButton.whileTrue(setWheelsToXMode);

        var scoringPositionModeButton = oi.driverGamepad.getifAvailable(XboxButton.Y);
        chordFactory.create(
                scoringPositionModeButton,
                povDown
        ).whileTrue(swerveNearestScoringProvider.get());
        var swerveNearestGamePieceScoringPosition = swerveNearestScoringProvider.get();
        swerveNearestGamePieceScoringPosition.setSpecificGamePiece(true);
        chordFactory.create(
                scoringPositionModeButton,
                povUp
        ).whileTrue(swerveNearestGamePieceScoringPosition);
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

        /*
        StartEndCommand activateJustPrecisionRotation = new StartEndCommand(
                () -> drive.setPrecisionRotationActive(true),
                () -> drive.setPrecisionRotationActive(false));
         */
        scoringPositionModeButton.whileTrue(drive.createEnableDisableQuickAlignActive());
    }

    @Inject
    public void setupAutonomousDriveCommands(
            OperatorInterface oi,
            VelocityMaintainerCommand velocityMaintainer,
            AutoBalanceCommand balanceCommand) {
        //oi.driverGamepad.getXboxButton(XboxButton.Start).whileTrue(balanceCommand);
        //oi.driverGamepad.getXboxButton(XboxButton.B).onTrue(velocityMaintainer);
    }

    @Inject
    public void setupGeneralSwerveCommands(SetSwerveMotorControllerPidParametersCommand setSteeringPidValues) {
        //setSteeringPidValues.includeOnSmartDashboard("Commit steering pid values");
    }

    @Inject
    public void setupMobilityCommands(OperatorInterface oi,
                                      TurnLeft90DegreesCommand turnleft90,
                                      SwerveToPointCommand swerveToPoint,

                                      DriveSubsystem drive,
                                      PropertyFactory pf) {

        pf.setPrefix("OperatorCommandMap/");

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
                                        AutonomousOracle oracle,
                                        ScoreCubeMidCommandGroup scoreCubeMid,
                                        ScoreConeHighCommandGroup scoreConeHigh,
                                        ChordTrigger.ChordTriggerFactory chordTriggerFactory) {

        var scoreCubeMidThenStop = setAutonomousCommandProvider.get();
        scoreCubeMidThenStop.setAutoCommand(scoreCubeMid);
        scoreCubeMidThenStop.includeOnSmartDashboard("AutoPrograms/ScoreCubeMidThenStop");
        //oi.experimentalInput.getifAvailable(XboxButton.LeftBumper).onTrue(scoreCubeMidThenStop);

        var scoreConeHighThenStop = setAutonomousCommandProvider.get();
        scoreConeHighThenStop.setAutoCommand(scoreConeHigh);
        scoreConeHighThenStop.includeOnSmartDashboard("AutoPrograms/ScoreConeHighThenStop");
        //oi.experimentalInput.getifAvailable(XboxButton.RightBumper).onTrue(scoreConeHighThenStop);
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

        oi.experimentalInput.getPovIfAvailable(0).onTrue(setPositionFiveToBalance);
        oi.experimentalInput.getPovIfAvailable(90).onTrue(setPositionFiveMobilityThenBalance);
        oi.experimentalInput.getPovIfAvailable(180).onTrue(setScoreCubeHighThenLeave);
        oi.experimentalInput.getPovIfAvailable(270).onTrue(setScoreCubeHighThenBalance);

        // These four programs seem reliable, and are based on the above programs but without further testing I'm concerned.
        // Should be prioritized for testing, especially the basic eject & balance combo.
        // Mapping the four of these to buttons on the gamepad.

        var setEjectLowThenBalance = setAutonomousCommandProvider.get();
        setEjectLowThenBalance.setAutoCommand(ejectLowThenBalance);
        //setEjectLowThenBalance.includeOnSmartDashboard("AutoPrograms/SetEjectLowThenBalance");

        var setEjectLowThenBalanceWithMobility = setAutonomousCommandProvider.get();
        setEjectLowThenBalanceWithMobility.setAutoCommand(ejectLowThenBalanceWithMobility);
        //setEjectLowThenBalanceWithMobility.includeOnSmartDashboard("AutoPrograms/SetEjectLowThenBalanceWithMobility");

        var setEjectLowThenExitLow = setAutonomousCommandProvider.get();
        setEjectLowThenExitLow.setAutoCommand(ejectLowThenExitLow);
        //setEjectLowThenExitLow.includeOnSmartDashboard("AutoPrograms/SetEjectLowThenExitLow");

        var setEjectLowThenExitHigh = setAutonomousCommandProvider.get();
        setEjectLowThenExitHigh.setAutoCommand(ejectLowThenExitHigh);
        //setEjectLowThenExitHigh.includeOnSmartDashboard("AutoPrograms/SetEjectLowThenExitHigh");

        //oi.experimentalInput.getifAvailable(XboxButton.A).onTrue(setEjectLowThenBalance);
        //oi.experimentalInput.getifAvailable(XboxButton.B).onTrue(setEjectLowThenBalanceWithMobility);
        //oi.experimentalInput.getifAvailable(XboxButton.X).onTrue(setEjectLowThenExitLow);
        //oi.experimentalInput.getifAvailable(XboxButton.Y).onTrue(setEjectLowThenExitHigh);

        // This is highly experimental, but gives the drive team a lot of flexibility with alliance partners.

        var setParameterizedAutonomousProgram = setAutonomousCommandProvider.get();
        setParameterizedAutonomousProgram.setAutoCommand(parameterizedAutonomousProgram);
        setParameterizedAutonomousProgram.includeOnSmartDashboard("AutoPrograms/SetParameterizedAutonomousProgram");

        oi.experimentalInput.getifAvailable(28).onTrue(oracle.createFavoriteAutoOne()); // Z
        oi.experimentalInput.getifAvailable(29).onTrue(oracle.createFavoriteAutoTwo()); // X
        oi.experimentalInput.getifAvailable(30).onTrue(oracle.createFavoriteAutoThree()); // C
        oi.experimentalInput.getifAvailable(31).onTrue(oracle.createFavoriteAutoFour()); // V
        oi.experimentalInput.getifAvailable(32).onTrue(setParameterizedAutonomousProgram); // B

        // -----------------------------------------
        // Time for a giant pile of chords to configure the auto program
        // -----------------------------------------

        var enableButton = oi.experimentalInput.getifAvailable(10); // Q
        var disableButton = oi.experimentalInput.getifAvailable(11); // W
        // each of the major toggles
        var drivePhaseOneButton = oi.experimentalInput.getifAvailable(12); // E
        var acquireGamePieceButton = oi.experimentalInput.getifAvailable(13); // R
        var moveToScoreButton = oi.experimentalInput.getifAvailable(14); // T
        var scoreSecondGamePieceButton = oi.experimentalInput.getifAvailable(15); // Y
        var balanceButton = oi.experimentalInput.getifAvailable(16); // U
        // chord them together
        // On
        chordTriggerFactory.create(enableButton, drivePhaseOneButton).onTrue(
                new InstantCommand(() -> oracle.setEnableDrivePhaseOne(true)).ignoringDisable(true));
        chordTriggerFactory.create(enableButton, acquireGamePieceButton).onTrue(
                new InstantCommand(() -> oracle.setEnableAcquireGamePiece(true)).ignoringDisable(true));
        chordTriggerFactory.create(enableButton, moveToScoreButton).onTrue(
                new InstantCommand(() -> oracle.setEnableMoveToScore(true)).ignoringDisable(true));
        chordTriggerFactory.create(enableButton, scoreSecondGamePieceButton).onTrue(
                new InstantCommand(() -> oracle.setEnableSecondScore(true)).ignoringDisable(true));
        chordTriggerFactory.create(enableButton, balanceButton).onTrue(
                new InstantCommand(() -> oracle.setEnableBalance(true)).ignoringDisable(true));
        // Off
        chordTriggerFactory.create(disableButton, drivePhaseOneButton).onTrue(
                new InstantCommand(() -> oracle.setEnableDrivePhaseOne(false)).ignoringDisable(true));
        chordTriggerFactory.create(disableButton, acquireGamePieceButton).onTrue(
                new InstantCommand(() -> oracle.setEnableAcquireGamePiece(false)).ignoringDisable(true));
        chordTriggerFactory.create(disableButton, moveToScoreButton).onTrue(
                new InstantCommand(() -> oracle.setEnableMoveToScore(false)).ignoringDisable(true));
        chordTriggerFactory.create(disableButton, scoreSecondGamePieceButton).onTrue(
                new InstantCommand(() -> oracle.setEnableSecondScore(false)).ignoringDisable(true));
        chordTriggerFactory.create(disableButton, balanceButton).onTrue(
                new InstantCommand(() -> oracle.setEnableBalance(false)).ignoringDisable(true));

        // lane is the next most important, should probably be new row
        oi.experimentalInput.getifAvailable(19).onTrue(oracle.createSetLaneCommand(AutonomousOracle.Lane.Top));
        oi.experimentalInput.getifAvailable(20).onTrue(oracle.createSetLaneCommand(AutonomousOracle.Lane.Middle));
        oi.experimentalInput.getifAvailable(21).onTrue(oracle.createSetLaneCommand(AutonomousOracle.Lane.Bottom));

        // Next is positions. Can use the last two keys on the qwerty row for Initial and Second scoring positions.
        var initialShiftButton = oi.experimentalInput.getifAvailable(17); // I
        var secondShiftButton = oi.experimentalInput.getifAvailable(18); // O
        // Now buttons 1-9 (mapped to 1-9, thankfully)
        var oneButton = oi.experimentalInput.getifAvailable(1); // 1
        var twoButton = oi.experimentalInput.getifAvailable(2); // 2
        var threeButton = oi.experimentalInput.getifAvailable(3); // 3
        var fourButton = oi.experimentalInput.getifAvailable(4); // 4
        var fiveButton = oi.experimentalInput.getifAvailable(5); // 5
        var sixButton = oi.experimentalInput.getifAvailable(6); // 6
        var sevenButton = oi.experimentalInput.getifAvailable(7); // 7
        var eightButton = oi.experimentalInput.getifAvailable(8); // 8
        var nineButton = oi.experimentalInput.getifAvailable(9); // 9
        // Chord them together
        // Initial
        chordTriggerFactory.create(initialShiftButton, oneButton).onTrue(oracle.createInitialScoringPositionCommand(1));
        chordTriggerFactory.create(initialShiftButton, twoButton).onTrue(oracle.createInitialScoringPositionCommand(2));
        chordTriggerFactory.create(initialShiftButton, threeButton).onTrue(oracle.createInitialScoringPositionCommand(3));
        chordTriggerFactory.create(initialShiftButton, fourButton).onTrue(oracle.createInitialScoringPositionCommand(4));
        chordTriggerFactory.create(initialShiftButton, fiveButton).onTrue(oracle.createInitialScoringPositionCommand(5));
        chordTriggerFactory.create(initialShiftButton, sixButton).onTrue(oracle.createInitialScoringPositionCommand(6));
        chordTriggerFactory.create(initialShiftButton, sevenButton).onTrue(oracle.createInitialScoringPositionCommand(7));
        chordTriggerFactory.create(initialShiftButton, eightButton).onTrue(oracle.createInitialScoringPositionCommand(8));
        chordTriggerFactory.create(initialShiftButton, nineButton).onTrue(oracle.createInitialScoringPositionCommand(9));
        // Second
        chordTriggerFactory.create(secondShiftButton, oneButton).onTrue(oracle.createSecondScoringPositionCommand(1));
        chordTriggerFactory.create(secondShiftButton, twoButton).onTrue(oracle.createSecondScoringPositionCommand(2));
        chordTriggerFactory.create(secondShiftButton, threeButton).onTrue(oracle.createSecondScoringPositionCommand(3));
        chordTriggerFactory.create(secondShiftButton, fourButton).onTrue(oracle.createSecondScoringPositionCommand(4));
        chordTriggerFactory.create(secondShiftButton, fiveButton).onTrue(oracle.createSecondScoringPositionCommand(5));
        chordTriggerFactory.create(secondShiftButton, sixButton).onTrue(oracle.createSecondScoringPositionCommand(6));
        chordTriggerFactory.create(secondShiftButton, sevenButton).onTrue(oracle.createSecondScoringPositionCommand(7));
        chordTriggerFactory.create(secondShiftButton, eightButton).onTrue(oracle.createSecondScoringPositionCommand(8));
        chordTriggerFactory.create(secondShiftButton, nineButton).onTrue(oracle.createSecondScoringPositionCommand(9));

        // Game pieces
        var coneButton = oi.experimentalInput.getifAvailable(22); // F
        var cubeButton = oi.experimentalInput.getifAvailable(23); // G
        // Chords for initial and second
        chordTriggerFactory.create(initialShiftButton, coneButton).onTrue(oracle.createSetInitialGamePieceModeCommand(UnifiedArmSubsystem.GamePieceMode.Cone));
        chordTriggerFactory.create(initialShiftButton, cubeButton).onTrue(oracle.createSetInitialGamePieceModeCommand(UnifiedArmSubsystem.GamePieceMode.Cube));
        chordTriggerFactory.create(secondShiftButton, coneButton).onTrue(oracle.createSecondGamePieceModeCommand(UnifiedArmSubsystem.GamePieceMode.Cone));
        chordTriggerFactory.create(secondShiftButton, cubeButton).onTrue(oracle.createSecondGamePieceModeCommand(UnifiedArmSubsystem.GamePieceMode.Cube));

        // Scoring modes
        var scoreHighButton = oi.experimentalInput.getifAvailable(24); // H
        var scoreEjectButton = oi.experimentalInput.getifAvailable(25); // J
        // Chords for initial and second
        chordTriggerFactory.create(initialShiftButton, scoreHighButton).onTrue(
                oracle.createInitialScoringModeCommand(AutonomousOracle.ScoringMode.High));
        chordTriggerFactory.create(initialShiftButton, scoreEjectButton).onTrue(
                oracle.createInitialScoringModeCommand(AutonomousOracle.ScoringMode.Eject));
        chordTriggerFactory.create(secondShiftButton, scoreHighButton).onTrue(
                oracle.createSecondScoringModeCommand(AutonomousOracle.ScoringMode.High));
        chordTriggerFactory.create(secondShiftButton, scoreEjectButton).onTrue(
                oracle.createSecondScoringModeCommand(AutonomousOracle.ScoringMode.Eject));

        // Mantle prep position
        oi.experimentalInput.getifAvailable(26).onTrue(
                oracle.createMantlePrepPositionCommand(AutonomousOracle.MantlePrepPosition.InsideCommunity)); // K
        oi.experimentalInput.getifAvailable(27).onTrue(
                oracle.createMantlePrepPositionCommand(AutonomousOracle.MantlePrepPosition.OutsideCommunity)); // L

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
        SimpleXZRouterCommand armToStartingPosition = simpleXZRouterCommandProvider.get();
        armToStartingPosition.setKeyPointFromKeyArmPosition(
                UnifiedArmSubsystem.KeyArmPosition.StartingPosition, UnifiedArmSubsystem.RobotFacing.Forward);
        armToStartingPosition.includeOnSmartDashboard("Arm to starting position");

        oi.operatorGamepad.getifAvailable(XboxButton.Back).onTrue(setConeMode);
        oi.operatorGamepad.getifAvailable(XboxButton.Start).onTrue(setCubeMode);

        router.setTarget(UnifiedArmSubsystem.KeyArmPosition.MidGoal, UnifiedArmSubsystem.RobotFacing.Forward);


        var moveArmForDoubleSubstation = simpleXZRouterCommandProvider.get();
        moveArmForDoubleSubstation.setKeyPointFromKeyArmPosition(KeyArmPosition.LoadingTray, RobotFacing.Forward);
        var intakeCommand = gripperMotorSubsystem.createIntakeCommand();

        //var collectFromDoubleSubstation = moveArmForDoubleSubstation.alongWith(intakeCommand, closeClaw);
        var intakeAndCloseClaw = intakeCommand.alongWith(closeClaw);
        oi.operatorGamepad.getifAvailable(XboxButton.RightBumper).whileTrue(intakeAndCloseClaw);
        //reverse motor
        oi.operatorGamepad.getifAvailable(XboxButton.LeftBumper).whileTrue(gripperMotorSubsystem.setEject(-1.0));

        oi.operatorGamepad.getifAvailable(XboxButton.RightTrigger).whileTrue(collector.getCollectThenAutomaticallyRetractCommand());
        oi.operatorGamepad.getifAvailable(XboxButton.LeftTrigger).whileTrue(collector.getEjectThenStopCommand());

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
