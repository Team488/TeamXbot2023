package competition.subsystems.drive.commands;

import javax.inject.Inject;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import xbot.common.command.BaseCommand;
import xbot.common.math.PIDManager;
import xbot.common.math.XYPair;
import xbot.common.math.PIDManager.PIDManagerFactory;

public class PositionMaintainerCommand extends BaseCommand {

    private final DriveSubsystem drive;
    private final PoseSubsystem pose;

    private final PIDManager xPIDManager;
    
    @Inject
    public PositionMaintainerCommand(DriveSubsystem drive, PoseSubsystem pose, PIDManagerFactory pidFactory) {
        this.drive = drive;
        this.pose = pose;
        this.xPIDManager = pidFactory.create("DrivePositionMaintainer", 0.01, 0, 0, 0.1, -0.1);
    }

    public void initialize() {
        drive.setPositionMaintainerXTarget(pose.getFieldOrientedTotalDistanceTraveled().x);
        this.xPIDManager.reset();
    }

    public void execute() {
        double currentXPosition = pose.getFieldOrientedTotalDistanceTraveled().x;
        double output = xPIDManager.calculate(drive.getPositionMaintainerXTarget(), currentXPosition);
        drive.setVelocityMaintainerXTarget(output);
    }
}
    