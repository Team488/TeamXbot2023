package competition.subsystems.arm;

import competition.BaseCompetitionTest;
import competition.operator_interface.OperatorInterface;
import competition.subsystems.arm.commands.ControlArmsWithJoyStickCommand;
import static org.junit.Assert.assertEquals;

import edu.wpi.first.wpilibj.MockXboxControllerAdapter;
import org.junit.Test;
import xbot.common.controls.sensors.mock_adapters.MockAbsoluteEncoder;
import xbot.common.controls.sensors.mock_adapters.MockDutyCycleEncoder;

public class ControlArmsWithJoyStickCommandTest extends BaseCompetitionTest {
    LowerArmSegment lowerArm;
    UpperArmSegment upperArm;
    OperatorInterface oi;
    ControlArmsWithJoyStickCommand controlArmsWithJoyStickCommand;
    UnifiedArmSubsystem arms;

    @Override
    public void setUp() {
        super.setUp();
        lowerArm = getInjectorComponent().lowerArmSegment();
        upperArm = getInjectorComponent().upperArmSegment();
        controlArmsWithJoyStickCommand = getInjectorComponent().controlArmsWithJoyStickCommand();
        oi = getInjectorComponent().operatorInterface();
        arms = getInjectorComponent().unifiedArmSubsystem();

        arms.setBrake(false);

        arms.upperArm.setAbsoluteEncoderOffsetInDegrees(0);
        arms.lowerArm.setAbsoluteEncoderOffsetInDegrees(0);

        ((MockDutyCycleEncoder)arms.lowerArm.absoluteEncoder).setRawPosition(90.0/360.0);
        ((MockDutyCycleEncoder)arms.upperArm.absoluteEncoder).setRawPosition(-90.0/360.0);
    }

    @Test
    public void testArmsGoForward(){
        // Check arms are not moving
        checkArmPowers(0,0);

        controlArmsWithJoyStickCommand.initialize();

        // Check lower arm is moving forward
        ((MockXboxControllerAdapter)oi.operatorGamepad).setLeftStick(0,1);
        controlArmsWithJoyStickCommand.execute();

        //Check upper arm is moving forward
        ((MockXboxControllerAdapter)oi.operatorGamepad).setRightStick(0,1);
        controlArmsWithJoyStickCommand.execute();
        checkArmPowers(1,1);

        // Check lower arm is moving backwards
        ((MockXboxControllerAdapter)oi.operatorGamepad).setLeftStick(0,-1);
        controlArmsWithJoyStickCommand.execute();

        // Check upper arm is moving backwards
        ((MockXboxControllerAdapter)oi.operatorGamepad).setRightStick(0,-1);
        controlArmsWithJoyStickCommand.execute();
        checkArmPowers(-1,-1);
    }
    private void checkArmPowers(double lowerArmPower,double upperArmPower){
        // Only check the right motors, as they are the "leaders".
        assertEquals(upperArmPower,upperArm.rightMotor.get(), 0.001);
        assertEquals(lowerArmPower,lowerArm.rightMotor.get(), 0.001);
    }
}
