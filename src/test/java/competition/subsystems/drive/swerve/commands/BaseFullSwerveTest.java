package competition.subsystems.drive.swerve.commands;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Ignore;

import competition.BaseCompetitionTest;
import competition.operator_interface.OperatorInterface;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import xbot.common.controls.sensors.mock_adapters.MockCANCoder;
import xbot.common.subsystems.pose.BasePoseSubsystem;

@Ignore
public abstract class BaseFullSwerveTest extends BaseCompetitionTest{
    
    DriveSubsystem drive;
    OperatorInterface oi;
    PoseSubsystem pose;
    
    @Override
    public void setUp() {
        super.setUp();
        drive = (DriveSubsystem)getInjectorComponent().driveSubsystem();
        oi = getInjectorComponent().operatorInterface();
        pose = (PoseSubsystem)getInjectorComponent().poseSubsystem();
        
        pose.setCurrentHeading(BasePoseSubsystem.FACING_AWAY_FROM_DRIVERS);
        setAllSteeringModuleAngles(0);
        drive.refreshDataFrame();
    }

    protected void setAllSteeringModuleAngles(double angle) {
        ((MockCANCoder)(drive.getFrontLeftSwerveModuleSubsystem().getSteeringSubsystem().getEncoder())).setAbsolutePosition(angle);
        ((MockCANCoder)(drive.getFrontRightSwerveModuleSubsystem().getSteeringSubsystem().getEncoder())).setAbsolutePosition(angle);
        ((MockCANCoder)(drive.getRearLeftSwerveModuleSubsystem().getSteeringSubsystem().getEncoder())).setAbsolutePosition(angle);
        ((MockCANCoder)(drive.getRearRightSwerveModuleSubsystem().getSteeringSubsystem().getEncoder())).setAbsolutePosition(angle);
    }

    protected void checkAllModuleAngle(double angle) {
        assertEquals(angle, drive.getFrontLeftSwerveModuleSubsystem().getTargetState().angle.getDegrees(), 0.001);
        assertEquals(angle, drive.getFrontRightSwerveModuleSubsystem().getTargetState().angle.getDegrees(), 0.001);
        assertEquals(angle, drive.getRearLeftSwerveModuleSubsystem().getTargetState().angle.getDegrees(), 0.001);
        assertEquals(angle, drive.getRearRightSwerveModuleSubsystem().getTargetState().angle.getDegrees(), 0.001);
    }

    protected void checkAllModulePower(double power) {
        assertEquals(power, drive.getFrontLeftSwerveModuleSubsystem().getTargetState().speedMetersPerSecond
                * PoseSubsystem.INCHES_IN_A_METER / drive.getMaxTargetSpeedInchesPerSecond(), 0.001);
        assertEquals(power, drive.getFrontRightSwerveModuleSubsystem().getTargetState().speedMetersPerSecond
                * PoseSubsystem.INCHES_IN_A_METER / drive.getMaxTargetSpeedInchesPerSecond(), 0.001);
        assertEquals(power, drive.getRearLeftSwerveModuleSubsystem().getTargetState().speedMetersPerSecond
                * PoseSubsystem.INCHES_IN_A_METER / drive.getMaxTargetSpeedInchesPerSecond(), 0.001);
        assertEquals(power, drive.getRearRightSwerveModuleSubsystem().getTargetState().speedMetersPerSecond
                * PoseSubsystem.INCHES_IN_A_METER / drive.getMaxTargetSpeedInchesPerSecond(), 0.001);
    }

    protected void checkRobotTurning(boolean turningLeft) {
        assertTrue(turningLeft == drive.getFrontLeftSwerveModuleSubsystem().getTargetState().speedMetersPerSecond < 0);
        assertTrue(turningLeft == drive.getFrontRightSwerveModuleSubsystem().getTargetState().speedMetersPerSecond > 0);
        assertTrue(turningLeft == drive.getRearLeftSwerveModuleSubsystem().getTargetState().speedMetersPerSecond < 0);
        assertTrue(turningLeft == drive.getRearRightSwerveModuleSubsystem().getTargetState().speedMetersPerSecond > 0);
    }

    protected void checkAllModulesGoingForward(boolean forward) {
        assertEquals(forward, drive.getFrontLeftSwerveModuleSubsystem().getTargetState().speedMetersPerSecond > 0);
        assertEquals(forward, drive.getFrontRightSwerveModuleSubsystem().getTargetState().speedMetersPerSecond > 0);
        assertEquals(forward, drive.getRearLeftSwerveModuleSubsystem().getTargetState().speedMetersPerSecond > 0);
        assertEquals(forward, drive.getRearRightSwerveModuleSubsystem().getTargetState().speedMetersPerSecond > 0);
    }
}
