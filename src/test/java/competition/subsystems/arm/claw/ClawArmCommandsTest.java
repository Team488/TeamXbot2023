package competition.subsystems.arm.claw;

import competition.BaseCompetitionTest;
import competition.operator_interface.OperatorInterface;
import competition.subsystems.ClawArm.ClawSubsystem;
import competition.subsystems.ClawArm.CloseClawCommand;
import competition.subsystems.ClawArm.OpenClawCommand;
import edu.wpi.first.wpilibj.MockSolenoid;
import org.junit.Assert;
import org.junit.Test;

public class ClawArmCommandsTest extends BaseCompetitionTest {

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
    public void openCLawTest(){
        setCloseClaw();
        openClaw.initialize();
        openClaw.execute();
        checkCLawState(true);

    }
    @Test
    public void cLoseCLawTest(){
        setOpenClaw();
        closeClaw.initialize();
        closeClaw.execute();
        checkCLawState(false);
    }

    @Test
    public void openCLawTwiceTest(){
        setCloseClaw();
        openClaw.initialize();
        openClaw.execute();
        openClaw.execute();
        checkCLawState(true);

    }
    @Test
    public void cLoseCLawTwiceTest(){
        setOpenClaw();
        closeClaw.initialize();
        closeClaw.execute();
        closeClaw.execute();
        checkCLawState(false);
    }

    @Test
    public void openToCloseTest(){
        setCloseClaw();
        openClaw.initialize();
        openClaw.execute();
        closeClaw.initialize();
        closeClaw.execute();
        checkCLawState(false);

    }
    @Test
    public void cLoseToOpenTest(){
        setOpenClaw();
        closeClaw.initialize();
        closeClaw.execute();
        openClaw.initialize();
        openClaw.execute();
        checkCLawState(true);
    }



    private void checkCLawState(boolean clawState){
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