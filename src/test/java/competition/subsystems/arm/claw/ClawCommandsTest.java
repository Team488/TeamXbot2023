package competition.subsystems.arm.claw;

import competition.BaseCompetitionTest;
import competition.operator_interface.OperatorInterface;
import competition.subsystems.arm.UnifiedArmSubsystem;
import competition.subsystems.claw.ClawSubsystem;
import competition.subsystems.claw.CloseClawCommand;
import competition.subsystems.claw.OpenClawCommand;
import edu.wpi.first.wpilibj.MockSolenoid;
import org.junit.Assert;
import org.junit.Test;
import xbot.common.controls.sensors.mock_adapters.MockDutyCycleEncoder;

public class ClawCommandsTest extends BaseCompetitionTest {

    ClawSubsystem clawSubsystem;
    OperatorInterface oi;
    CloseClawCommand closeClaw;
    OpenClawCommand openClaw;
    UnifiedArmSubsystem arms;

    @Override
    public void setUp() {
        super.setUp();
        clawSubsystem = getInjectorComponent().clawSubsystem();
        closeClaw = getInjectorComponent().closeClawCommand();
        openClaw = getInjectorComponent().openClawCommand();
        arms = getInjectorComponent().unifiedArmSubsystem();
        arms.setIsCalibrated(true);
        setUpperArmAngles(90);
        arms.upperArm.setAbsoluteEncoderOffsetInDegrees(0);
        arms.lowerArm.setAbsoluteEncoderOffsetInDegrees(0);

        arms.refreshDataFrame();
    }

    @Test
    public void openClawTest() {
        setCloseClaw();
        openClaw.initialize();
        openClaw.execute();
        timer.advanceTimeInSecondsBy(1);
        openClaw.execute();
        checkClawState(true);
    }

    @Test
    public void openClawTestWithSafety() {
        setCloseClaw();
        openClaw.initialize();
        openClaw.execute();
        timer.advanceTimeInSecondsBy(1);
        openClaw.execute();
        checkClawState(true);
        setUpperArmAngles(0);
        openClaw.execute();
        checkClawState(false);
        setUpperArmAngles(90);
        openClaw.execute();
        checkClawState(false);
        timer.advanceTimeInSecondsBy(1);
        openClaw.execute();
        checkClawState(true);
    }

    @Test
    public void closeClawTest() {
        setOpenClaw();
        closeClaw.initialize();
        closeClaw.execute();
        closeClaw.execute();
        checkClawState(false);
    }

    @Test
    public void openClawTwiceTest() {
        setCloseClaw();
        openClaw.initialize();
        openClaw.execute();
        timer.advanceTimeInSecondsBy(1);
        openClaw.execute();
        checkClawState(true);

    }

    @Test
    public void closeClawTwiceTest() {
        setOpenClaw();
        closeClaw.initialize();
        closeClaw.execute();
        closeClaw.execute();
        checkClawState(false);
    }

    @Test
    public void openToCloseTest() {
        setCloseClaw();
        openClaw.initialize();
        openClaw.execute();
        timer.advanceTimeInSecondsBy(1);
        openClaw.execute();
        closeClaw.initialize();
        closeClaw.execute();
        checkClawState(false);

    }

    @Test
    public void closeToOpenTest() {
        setOpenClaw();
        closeClaw.initialize();
        closeClaw.execute();
        openClaw.initialize();
        openClaw.execute();
        timer.advanceTimeInSecondsBy(1);
        openClaw.execute();
        checkClawState(true);
    }

    private void checkClawState(boolean clawState) {
        Assert.assertEquals(clawState, clawSubsystem.getClawState());
        Assert.assertEquals(clawState, ((MockSolenoid) clawSubsystem.clawSolenoid).getAdjusted());
    }

    private void setOpenClaw() {
        clawSubsystem.open();
        ((MockSolenoid) clawSubsystem.clawSolenoid).set(true);
    }

    private void setCloseClaw() {
        clawSubsystem.close();
        ((MockSolenoid) clawSubsystem.clawSolenoid).set(false);
    }

    private void setUpperArmAngles(double upperArmAngle) {
        ((MockDutyCycleEncoder)arms.upperArm.absoluteEncoder).setRawPosition(upperArmAngle/360.0);
        arms.refreshDataFrame();
    }
}