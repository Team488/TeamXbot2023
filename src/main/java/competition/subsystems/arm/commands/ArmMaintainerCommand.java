package competition.subsystems.arm.commands;

import competition.operator_interface.OperatorInterface;
import competition.subsystems.arm.LowerArmSubsystem;
import competition.subsystems.arm.UpperArmSubsystem;
import xbot.common.command.BaseCommand;
import xbot.common.logic.HumanVsMachineDecider;
import xbot.common.math.PID;
import xbot.common.math.PIDManager;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

public class ArmMaintainerCommand extends BaseCommand {
    final OperatorInterface oi;
    final LowerArmSubsystem lowerArm;
    final UpperArmSubsystem upperArm;
    final PIDManager positionPid;
    final PIDManager velocityPid;
    final DoubleProperty maxVelocityProp;
    final HumanVsMachineDecider decider;
    final DoubleProperty currentTickGoal;
    final DoubleProperty currentPower;
    public double  maxVelocity;
    double previousTickPosition;


    public ArmMaintainerCommand(OperatorInterface oi, LowerArmSubsystem lowerArm, UpperArmSubsystem upperArm,
                                PIDManager.PIDManagerFactory pidFactory, PropertyFactory propertyFactory){
        this.oi = oi;
        this.lowerArm = lowerArm;
        this.upperArm = upperArm;

        this.positionPid = pidFactory.create(getPrefix() + "positionPID",0,0,0);
        this.velocityPid = pidFactory.create(getPrefix() + "velocityPID",0,0,0);

        this.requires(lowerArm);
        this.requires(upperArm);

        propertyFactory.setPrefix(this.getPrefix());
        maxVelocityProp = propertyFactory.createPersistentProperty("MaxVelocity",maxVelocity);
        // decider = clf.createHumanVsMachineDecider(getPrefix() + "Decider");



        this.currentTickGoal = propertyFactory.createEphemeralProperty("CurrentTickGoal",0);
        currentPower = propertyFactory.createEphemeralProperty("CurrentPower",0);
    }
    @Override
    public void initialize(){
        log.info("initializing");

    }

    @Override
    public void execute() {

    }
}
