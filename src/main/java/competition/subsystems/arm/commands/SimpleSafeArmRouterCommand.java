package competition.subsystems.arm.commands;

import competition.subsystems.arm.UnifiedArmSubsystem;
import edu.wpi.first.math.Pair;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import xbot.common.command.BaseSetpointCommand;

import javax.inject.Inject;
import java.util.ArrayList;
import java.util.List;

public class SimpleSafeArmRouterCommand extends BaseSetpointCommand {

    private final UnifiedArmSubsystem arms;
    private List<Pair<UnifiedArmSubsystem.KeyArmPosition, UnifiedArmSubsystem.RobotFacing>> armPosesToVisit;
    private UnifiedArmSubsystem.KeyArmPosition targetArmPosition;
    private UnifiedArmSubsystem.RobotFacing targetRobotFacing;

    private static Logger log = LogManager.getLogger(SimpleSafeArmRouterCommand.class);

    @Inject
    public SimpleSafeArmRouterCommand(UnifiedArmSubsystem arms) {
        super(arms);
        this.arms = arms;
        armPosesToVisit = new ArrayList<>();
    }

    public void setTarget(UnifiedArmSubsystem.KeyArmPosition targetArmPosition, UnifiedArmSubsystem.RobotFacing targetRobotFacing) {
        this.targetArmPosition = targetArmPosition;
        this.targetRobotFacing = targetRobotFacing;
    }

    @Override
    public void initialize() {
        log.info("Initializing");
        UnifiedArmSubsystem.RobotFacing currentArmFacing = arms.getCurrentEndEffectorFacing();

        // Some basic rules:
        // 1) As long as we are on the same side, it should always be safe go to the SafeExternalTransition point.
        // 2) If we need to swap sides, we need to go to SafeExternalTransition on one side, then the other, before
        //    going to the target.
        // Eventually we should use a more sophisticated approach using some kind of configuration space to route
        // around potential obstacles, but we like to start simple and then optimize what needs to be optimized.

        // Given the rules above, it seems like we should always first go to the SafeExternalTransition point for
        // the side we are on, and then (if necessary) go to the SafeExternalTransition point for the other side,
        // before going to our final target point.

        armPosesToVisit.add(new Pair<>(UnifiedArmSubsystem.KeyArmPosition.SafeExternalTransition, currentArmFacing));
        if (currentArmFacing != targetRobotFacing) {
            log.info("Need to switch sides; adding a SafeExternalTransition for the other side");
            armPosesToVisit.add(new Pair<>(UnifiedArmSubsystem.KeyArmPosition.SafeExternalTransition, targetRobotFacing));
        }

        // If our goal is to go to the SafeExternalTransition point, then we don't need to add it to the list - it
        // was added by default!
        if (targetArmPosition != UnifiedArmSubsystem.KeyArmPosition.SafeExternalTransition) {

            armPosesToVisit.add(new Pair<>(targetArmPosition, targetRobotFacing));
        }

        // Then go ahead and set the initial target, so that execute can immediately start looking for that completion.
        setTargetFromFirstEntryInList();
        // Since we just set the target, the maintainer hasn't had a chance to execute yet and evaluate to see if we
        // are at that. To avoid premature completion, we will force the subsystem to say the maintainer
        // is not yet at the goal, since we will be checking that value immediately in execute.
        arms.setMaintainerIsAtGoal(false);
    }

    @Override
    public void execute() {
        if (armPosesToVisit.size() == 0) {
            // This shouldn't happen, but just in case, do nothing.
            return;
        }

        if (arms.isMaintainerAtGoal()) {
            // We are at the current target, so we can move on to the next one.
            armPosesToVisit.remove(0);

            if (armPosesToVisit.size() > 0) {
                // If there are more targets to visit, set the next one.
                setTargetFromFirstEntryInList();
            }
        }

        // Otherwise, we are still moving to the current target, so do nothing.
    }

    /**
     * Likely only has value in testing, though it could be used to expose a bit of state to a dashboard.
     * @return The list of planned positions to visit.
     */
    public List<Pair<UnifiedArmSubsystem.KeyArmPosition, UnifiedArmSubsystem.RobotFacing>> getArmPosesToVisit() {
        return armPosesToVisit;
    }

    private void setTargetFromFirstEntryInList() {
        log.info("Setting target to " + armPosesToVisit.get(0).getFirst() + " " + armPosesToVisit.get(0).getSecond());
        arms.setTargetValue(
                arms.getKeyArmAngles(
                        armPosesToVisit.get(0).getFirst(), // KeyArmPosition
                        armPosesToVisit.get(0).getSecond() // RobotFacing
                )
        );
    }

    @Override
    public boolean isFinished() {
        return armPosesToVisit.size() == 0;
    }
}
