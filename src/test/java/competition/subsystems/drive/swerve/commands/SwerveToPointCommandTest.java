package competition.subsystems.drive.swerve.commands;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import competition.subsystems.drive.commands.SwerveToPointCommand;
import xbot.common.math.WrappedRotation2d;
import xbot.common.math.XYPair;

public class SwerveToPointCommandTest extends BaseFullSwerveTest {
    
    SwerveToPointCommand command;

    @Override
    public void setUp() {
        super.setUp();
        command = getInjectorComponent().swerveToPointCommand();
        drive.getPositionalPid().setP(1);
        drive.getPositionalPid().setI(0);
        drive.getPositionalPid().setD(0);
    }

    @Test
    public void multiplePositionsTest() {

        pose.setCurrentPosition(0, 0);
        pose.setCurrentHeading(90);
        command.setTargetPosition(new XYPair(100, 100), 90);
        command.initialize();
        command.execute();

        checkAllModuleAngle(-45);
        checkAllModulesGoingForward(true);

        pose.setCurrentPosition(100, 0);
        command.execute();

        checkAllModuleAngle(0);
        checkAllModulesGoingForward(true);

        pose.setCurrentPosition(100, 150);
        command.execute();

        // Angles don't change due to swerve module optimization (no need to rotate if we can just drive backwards)
        checkAllModuleAngle(0);
        checkAllModulesGoingForward(false);
    }

    @Test
    public void robotSimpleRelativeMotionTest() {
        command.setRobotRelativeMotion();
        command.setTargetPosition(new XYPair(60, 0), 0);

        command.initialize();
        command.execute();

        checkAllModulesGoingForward(true);

        command.setTargetPosition(new XYPair(0,0), 0);

        command.initialize();
        command.execute();

        checkRobotTurning(false);
    }

    @Test
    public void robotOffsetRelativeMotionTest() {

        pose.setCurrentHeading(-45); 
        setAllSteeringModuleAngles(90);

        command.setRobotRelativeMotion();
        command.setTargetPosition(new XYPair(0, 60), 0);

        command.initialize();
        command.execute();

        checkAllModulesGoingForward(true);

        command.setTargetPosition(new XYPair(0,0), 0);
        setAllSteeringModuleAngles(90);

        command.initialize();
        command.execute();

        checkRobotTurning(false);
    }

    @Test
    public void autonomousTest() {

        pose.setCurrentPosition(0, -12);
        pose.setCurrentHeading(90);
        setAllSteeringModuleAngles(90);

        command.setRobotRelativeMotion();
        command.setTargetPosition(new XYPair(0, -60), -90);

        command.initialize();
        command.execute();

        pose.setCurrentPosition(-30,-40);

        command.execute();
    }

    @Test
    public void multipleInitTest() {
        pose.setCurrentPosition(0, 0);
        pose.setCurrentHeading(-90);
        setAllSteeringModuleAngles(90);

        command.setRobotRelativeMotion();
        command.setTargetPosition(new XYPair(0, -60), -45);

        command.initialize();
        command.execute();

        assertEquals(-60, command.getActiveTargetPosition().x, 0.01);
        assertEquals(0, command.getActiveTargetPosition().y, 0.01);
        assertEquals(-135, WrappedRotation2d.fromDegrees(command.getActiveHeading()).getDegrees(), 0.01);

        command.initialize();
        command.execute();

        assertEquals(-60, command.getActiveTargetPosition().x, 0.01);
        assertEquals(0, command.getActiveTargetPosition().y, 0.01);
        assertEquals(-135, WrappedRotation2d.fromDegrees(command.getActiveHeading()).getDegrees(), 0.01);
    }

}
