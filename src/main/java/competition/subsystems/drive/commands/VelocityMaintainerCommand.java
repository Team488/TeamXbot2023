package competition.subsystems.drive.commands;

import javax.inject.Inject;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import xbot.common.math.PIDManager;
import xbot.common.math.XYPair;
import xbot.common.math.PIDManager.PIDManagerFactory;

public class VelocityMaintainerCommand {

    private final DriveSubsystem drive;
    private final PoseSubsystem pose;

    private final PIDManager xPIDManager;

    // state variables
    private double xThrottle;
    
    @Inject
    public VelocityMaintainerCommand(DriveSubsystem drive, PoseSubsystem pose, PIDManagerFactory pidFactory) {
        this.drive = drive;
        this.pose = pose;
        this.xPIDManager = pidFactory.create("DriveVelocity", 0, 0, 0);
    }

    public void initialize() {
        this.xPIDManager.reset();
        this.xThrottle = 0;
    }

    public void execute() {
        double currentXVelocity = pose.getCurrentVelocity().x;
        double deltaPower = xPIDManager.calculate(drive.getVelocityMaintainerXTarget(), currentXVelocity);
        this.xThrottle += deltaPower;
        drive.move(new XYPair(xThrottle,0), 0);
    }
}
