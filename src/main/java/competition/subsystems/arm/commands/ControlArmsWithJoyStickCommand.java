package competition.subsystems.arm.commands;

import competition.operator_interface.OperatorInterface;
import competition.subsystems.arm.LowerArmSubsystem;
import competition.subsystems.arm.UpperArmSubsystem;
import xbot.common.command.BaseCommand;
import xbot.common.math.MathUtils;

import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public class ControlArmsWithJoyStickCommand extends BaseCommand {
    private final OperatorInterface oi;
    private final LowerArmSubsystem lowerArmSubsystem;
    private final UpperArmSubsystem upperArmSubsystem;
    @Inject
    public ControlArmsWithJoyStickCommand(OperatorInterface oi, LowerArmSubsystem lowerArmSubsystem, UpperArmSubsystem upperArmSubsystem ){
        this.oi = oi;
        this.lowerArmSubsystem = lowerArmSubsystem;
        this.upperArmSubsystem = upperArmSubsystem;

    }

    public void initialize(){

    }

    public void execute(){
        //Use left joystick to control lower arm
        double lowerArmPower = oi.operatorGamepad.getLeftStickY();
        // use right joystick to control upper arm
        double upperArmPower =  oi.operatorGamepad.getRightStickY();

        lowerArmPower = MathUtils.deadband(lowerArmPower, oi.getOperatorGamepadTypicalDeadband());
        upperArmPower = MathUtils.deadband(upperArmPower, oi.getOperatorGamepadTypicalDeadband());

        lowerArmSubsystem.setMotorPower(lowerArmPower);
        upperArmSubsystem.setMotorPower(upperArmPower);

    }
}
