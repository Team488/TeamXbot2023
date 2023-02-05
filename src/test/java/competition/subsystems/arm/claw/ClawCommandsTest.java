package competition.subsystems.arm.claw;

import competition.BaseCompetitionTest;
import competition.operator_interface.OperatorInterface;
import competition.subsystems.ClawArm.ClawSubsystem;
import competition.subsystems.ClawArm.CloseClawCommand;
import competition.subsystems.ClawArm.OpenClawCommand;
import edu.wpi.first.wpilibj.MockSolenoid;
import org.junit.Assert;
import org.junit.Test;

public class ClawCommandsTest extends BaseCompetitionTest {

    ClawSubsystem clawSubsystem;
    OperatorInterface oi;
    CloseClawCommand closeClaw;
    OpenClawCommand openClaw;

    @Override
    public void setUp() {
        super.setUp();
        clawSubsystem = getInjectorComponent().clawSubsystem();
        closeClaw = getInjectorComponent().closeClawCommand();
        openClaw = getInjectorComponent().openClawCommand();
    }

    @Test
    public void openClawTest(){
        setCloseClaw();
        openClaw.initialize();
        openClaw.execute();
        checkClawState(true);

    }
    @Test
    public void closeClawTest(){
        setOpenClaw();
        closeClaw.initialize();
        closeClaw.execute();
        checkClawState(false);
    }

    @Test
    public void openClawTwiceTest(){
        setCloseClaw();
        openClaw.initialize();
        openClaw.execute();
        openClaw.execute();
        checkClawState(true);

    }
    @Test
    public void cLoseClawTwiceTest(){
        setOpenClaw();
        closeClaw.initialize();
        closeClaw.execute();
        closeClaw.execute();
        checkClawState(false);
    }

    @Test
    public void openToCloseTest(){
        setCloseClaw();
        openClaw.initialize();
        openClaw.execute();
        closeClaw.initialize();
        closeClaw.execute();
        checkClawState(false);

    }
    @Test
    public void closeToOpenTest(){
        setOpenClaw();
        closeClaw.initialize();
        closeClaw.execute();
        openClaw.initialize();
        openClaw.execute();
        checkClawState(true);
    }



    private void checkClawState(boolean clawState){
        Assert.assertEquals(clawState, clawSubsystem.GetClawState());

        Assert.assertEquals(clawState, ((MockSolenoid)clawSubsystem.ClawSolenoid).getAdjusted());

    }

    private void setOpenClaw(){
        clawSubsystem.Open();
        ((MockSolenoid)clawSubsystem.ClawSolenoid).set(true);

    }

    private void setCloseClaw(){
        clawSubsystem.Close();
        ((MockSolenoid)clawSubsystem.ClawSolenoid).set(false);
    }
}