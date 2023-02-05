package competition.subsystems.drive.swerve.commands;

import competition.subsystems.drive.DriveSubsystem;
import org.junit.Test;

import competition.subsystems.drive.commands.SwerveDriveWithJoysticksCommand;
import edu.wpi.first.wpilibj.MockXboxControllerAdapter;
import xbot.common.math.XYPair;

public class SwerveDriveWithJoysticksCommandTest extends BaseFullSwerveTest {

    SwerveDriveWithJoysticksCommand command;
    DriveSubsystem drive;

    @Override
    public void setUp() {
        super.setUp();
        command = getInjectorComponent().swerveDriveWithJoysticksCommand();
        drive = (DriveSubsystem)getInjectorComponent().driveSubsystem();
    }

    @Test
    public void simpleTest() {
        ((MockXboxControllerAdapter)oi.driverGamepad).setLeftStick(new XYPair(0, 1));
        drive.refreshDataFrame();
        command.initialize();
        command.execute();

        checkAllModuleAngle(0);
        checkAllModulePower(1);

        ((MockXboxControllerAdapter)oi.driverGamepad).setLeftStick(new XYPair(1, 1));

        command.execute();

        checkAllModuleAngle(-45);
        checkAllModulePower(1);
    }

    @Test
    public void testAbsoluteHeading() {
        ((MockXboxControllerAdapter)oi.driverGamepad).setLeftStick(new XYPair(0, 0));
        ((MockXboxControllerAdapter)oi.driverGamepad).setRightStick(new XYPair(0, 1));
        timer.advanceTimeInSecondsBy(100);
        command.setAbsoluteHeadingMode(true);
        drive.refreshDataFrame();

        // at this point, a command of 0,1 should be interpreted as a goal of 90 degrees - meaning no motion,
        // since the robot is already at 90 degrees.
        command.initialize();
        //command.execute();

        //checkAllModulePower(0);

        // Point the joystick to 0 degrees
        ((MockXboxControllerAdapter)oi.driverGamepad).setRawRightStick(new XYPair(1, 0));
        command.execute();

        // Check for right turn (since we started at 90)
        checkRobotTurning(false);

        // Point the joystick to 180 degrees
        ((MockXboxControllerAdapter)oi.driverGamepad).setRawRightStick(new XYPair(-1, 0));
        command.execute();

        checkRobotTurning(true);

        // Check that we preserve the last high-magnitude value (in this case, the left turn).
        ((MockXboxControllerAdapter)oi.driverGamepad).setRawRightStick(new XYPair(0, 0));
        command.execute();
        checkRobotTurning(true);
    }
}