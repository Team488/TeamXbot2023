package competition.operator_interface;

import competition.auto_programs.BlueBottomScoringPath;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.drive.commands.AutoBalanceCommand;
import competition.subsystems.drive.commands.DebuggingSwerveWithJoysticksCommand;
import competition.subsystems.drive.commands.GoToNextActiveSwerveModuleCommand;
import competition.subsystems.drive.commands.SetSwerveMotorControllerPidParametersCommand;
import competition.subsystems.drive.commands.SwerveDriveWithJoysticksCommand;
import competition.subsystems.drive.commands.SwerveToPointCommand;
import competition.subsystems.drive.commands.TurnLeft90DegreesCommand;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.simple.SimpleSetPowerCommand;
import competition.subsystems.vision.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
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
    public OperatorCommandMap() {}
    
    @Inject
    public void setupDriveCommands(OperatorInterface oi,
            SetRobotHeadingCommand resetHeading,
            DriveSubsystem drive,
            PoseSubsystem pose,
            DebuggingSwerveWithJoysticksCommand debugSwerve,
            GoToNextActiveSwerveModuleCommand nextModule,
            SwerveDriveWithJoysticksCommand regularSwerve,
            VisionSubsystem vision) {
        resetHeading.setHeadingToApply(0);

        NamedInstantCommand resetPosition = new NamedInstantCommand("Reset Position",
                () -> pose.setCurrentPosition(0, 0));
        ParallelCommandGroup resetPose = new ParallelCommandGroup(resetPosition, resetHeading);

        StartEndCommand enableVisionRotation = new StartEndCommand(
                () -> drive.setRotateToHubActive(true),
                () -> drive.setRotateToHubActive(false));

        oi.driverGamepad.getifAvailable(XboxButton.A).onTrue(resetPose);
        oi.vjoyKeyboard.getifAvailable(3).onTrue(resetPose);
        oi.vjoyKeyboard.getifAvailable(4).onTrue(resetPose);
        oi.vjoyKeyboard.getifAvailable(5).onTrue(resetPose);

        oi.vjoyKeyboard.getifAvailable(19).onTrue(resetPose);
        oi.vjoyKeyboard.getifAvailable(20).onTrue(resetPose);
        oi.vjoyKeyboard.getifAvailable(21).onTrue(resetPose);

        oi.vjoyKeyboard.getifAvailable(54).onTrue(resetPose);
        oi.vjoyKeyboard.getifAvailable(55).onTrue(resetPose);
        oi.vjoyKeyboard.getifAvailable(56).onTrue(resetPose);


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
}
