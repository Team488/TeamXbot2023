package competition.subsystems.drive.commands;

import javax.inject.Inject;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import xbot.common.command.BaseCommand;
import xbot.common.math.PIDManager;
import xbot.common.math.XYPair;
import xbot.common.math.PIDManager.PIDManagerFactory;

public class AutoBalanceCommand extends BaseCommand {
    
    private final DriveSubsystem drive;
    private final PoseSubsystem pose;
    private final PIDManager pidManager;

    @Inject
    public AutoBalanceCommand(DriveSubsystem drive, PoseSubsystem pose, PIDManagerFactory pidFactory) {
        this.drive = drive;
        this.pose = pose;
        this.pidManager = pidFactory.create(this.getPrefix(), 0.1, 0, 0);
    }

    @Override
    public void initialize() {
        log.info("Initializing");
        pidManager.reset();
    }

    @Override
    public void execute() {
        double currentAngle = pose.getRobotPitch();

        if (Math.abs(currentAngle) < 1.5) {
            currentAngle = 0;
        }

        double velocity = pidManager.calculate(0, currentAngle);

        drive.setVelocityMaintainerXTarget(velocity);
    }
}
