package competition.subsystems.arm.commands;

import competition.operator_interface.OperatorInterface;
import competition.subsystems.arm.LowerArmSegment;
import competition.subsystems.arm.UnifiedArmSubsystem;
import competition.subsystems.arm.UpperArmSegment;
import xbot.common.command.BaseCommand;
import xbot.common.math.MathUtils;

import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public class ControlArmsWithJoyStickCommand extends BaseCommand {
    private final OperatorInterface oi;
    private final UnifiedArmSubsystem arms;
    @Inject
    public ControlArmsWithJoyStickCommand(OperatorInterface oi, UnifiedArmSubsystem arms){
        this.oi = oi;
        this.arms = arms;
        this.addRequirements(arms);
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

        arms.setArmPowers(lowerArmPower, upperArmPower);
    }
}
