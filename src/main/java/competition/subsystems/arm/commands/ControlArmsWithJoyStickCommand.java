package competition.subsystems.arm.commands;

import competition.operator_interface.OperatorInterface;
import competition.subsystems.arm.LowerArmSegment;
import competition.subsystems.arm.UpperArmSegment;
import xbot.common.command.BaseCommand;
import xbot.common.math.MathUtils;

import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public class ControlArmsWithJoyStickCommand extends BaseCommand {
    private final OperatorInterface oi;
    private final LowerArmSegment lowerArmSegment;
    private final UpperArmSegment upperArmSegment;
    @Inject
    public ControlArmsWithJoyStickCommand(OperatorInterface oi, LowerArmSegment lowerArmSegment, UpperArmSegment upperArmSegment){
        this.oi = oi;
        this.lowerArmSegment = lowerArmSegment;
        this.upperArmSegment = upperArmSegment;

    }

    public void initialize(){
        log.info("Initializing!");
    }

    public void execute(){
        //Use left joystick to control lower arm
        double lowerArmPower = oi.operatorGamepad.getLeftStickY();
        // use right joystick to control upper arm
        double upperArmPower =  oi.operatorGamepad.getRightStickY();

        lowerArmPower = MathUtils.deadband(lowerArmPower, oi.getOperatorGamepadTypicalDeadband());
        upperArmPower = MathUtils.deadband(upperArmPower, oi.getOperatorGamepadTypicalDeadband());

        lowerArmSegment.setPower(lowerArmPower);
        upperArmSegment.setPower(upperArmPower);

    }
}
