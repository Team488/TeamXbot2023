package competition.commandgroups;

import competition.subsystems.arm.UnifiedArmSubsystem;
import competition.subsystems.arm.commands.SimpleXZRouterCommand;
import competition.subsystems.collector.commands.CollectIfSafeCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import javax.inject.Inject;

public class CollectionSequenceCommandGroup extends ParallelCommandGroup {

    @Inject
    public CollectionSequenceCommandGroup(
            CollectIfSafeCommand collectIfSafe,
            SimpleXZRouterCommand prepareArmToPickupFromCollector
    ) {
        prepareArmToPickupFromCollector.setKeyPointFromKeyArmPosition(
                UnifiedArmSubsystem.KeyArmPosition.PrepareToAcquireFromCollector,
                UnifiedArmSubsystem.RobotFacing.Forward);

        this.addCommands(prepareArmToPickupFromCollector);
        this.addCommands(collectIfSafe);
    }
}
