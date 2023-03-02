package competition.subsystems.claw;

import competition.subsystems.arm.UnifiedArmSubsystem;
import xbot.common.command.BaseCommand;
import xbot.common.logic.TimeStableValidator;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;

public class OpenClawCommand extends BaseCommand{
    private final ClawSubsystem clawSubsystem;
    private final UnifiedArmSubsystem arm;
    private final TimeStableValidator timeStableValidator;

    private final DoubleProperty armSafetyThreshold;
    private final DoubleProperty clawSafeTimeThreshold;

    @Inject
    public OpenClawCommand(ClawSubsystem clawSubsystem, UnifiedArmSubsystem arm, PropertyFactory pf){
        this.clawSubsystem = clawSubsystem;
        this.arm = arm;

        pf.setPrefix(this);
        armSafetyThreshold = pf.createPersistentProperty("Arm Safety Limit Degrees", 10.0);
        clawSafeTimeThreshold = pf.createPersistentProperty("Arm Safety Time Threshold", 0.2);

        timeStableValidator = new TimeStableValidator(() -> clawSafeTimeThreshold.get());

        addRequirements(clawSubsystem);
    }

    @Override
    public void initialize() {
        // Nothing to do
        log.info("Initializing");
        timeStableValidator.setStable();
    }

    @Override
    public void execute() {
        // Only open the claw when it is safe to do so
        double currentArmPosition = arm.upperArm.getArmPositionInDegrees();
        boolean isSafe = Math.abs(currentArmPosition) > armSafetyThreshold.get();
        boolean isStable = timeStableValidator.checkStable(isSafe);
        if (isSafe && isStable) {
            clawSubsystem.open();
        } else {
            clawSubsystem.close();
        }
    }
}