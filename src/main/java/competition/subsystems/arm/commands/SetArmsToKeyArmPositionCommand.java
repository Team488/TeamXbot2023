package competition.subsystems.arm.commands;

import competition.subsystems.arm.UnifiedArmSubsystem;
import xbot.common.command.BaseSetpointCommand;
import java.util.function.Supplier;
import javax.inject.Inject;

public class SetArmsToKeyArmPositionCommand extends BaseSetpointCommand {

    UnifiedArmSubsystem arms;

    private Supplier<UnifiedArmSubsystem.KeyArmPosition> targetArmPositionSupplier;
    private Supplier<UnifiedArmSubsystem.RobotFacing> targetRobotFacingSupplier;

    @Inject
    public SetArmsToKeyArmPositionCommand(UnifiedArmSubsystem arms) {
        super(arms);
        this.arms = arms;
    }

    public void setTargetSupplier(Supplier<UnifiedArmSubsystem.KeyArmPosition> targetArmPositionSupplier,
                                  Supplier<UnifiedArmSubsystem.RobotFacing> targetRobotFacingSupplier) {
        this.targetArmPositionSupplier = targetArmPositionSupplier;
        this.targetRobotFacingSupplier = targetRobotFacingSupplier;
    }

    public void initialize() {
        var angles = arms.getKeyArmAngles(targetArmPositionSupplier.get(), targetRobotFacingSupplier.get());
        arms.setTargetValue(angles);
        arms.setMaintainerIsAtGoal(false);
    }

    @Override
    public void execute() {
        // No-op. Wait for the arms to get to the target.
    }

    @Override
    public boolean isFinished() {
        return arms.isMaintainerAtGoal();
    }
}
