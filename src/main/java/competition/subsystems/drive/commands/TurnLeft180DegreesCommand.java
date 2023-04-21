package competition.subsystems.drive.commands;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import xbot.common.command.BaseCommand;
import xbot.common.math.MathUtils;
import xbot.common.math.XYPair;
import xbot.common.subsystems.drive.control_logic.HeadingModule;

import javax.inject.Inject;

public class TurnLeft180DegreesCommand extends BaseCommand {
    private Rotation2d initialHeading;
    protected Rotation2d goalHeading;
    DriveSubsystem drive;
    PoseSubsystem pose;
    HeadingModule headingModule;
    double maxPower = 0.6;

    @Inject
    public TurnLeft180DegreesCommand(DriveSubsystem drive, PoseSubsystem pose, HeadingModule.HeadingModuleFactory headingModuleFactory){
        this.drive = drive;
        this.pose = pose;
        this.headingModule = headingModuleFactory.create(drive.getRotateToHeadingPid());

    }

    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }

    @Override
    public void initialize() {
        log.info("Initializing");
        initialHeading = pose.getCurrentHeading();
        goalHeading = initialHeading.plus(Rotation2d.fromDegrees(180));
    }

    @Override
    public void execute() {
        double power = headingModule.calculateHeadingPower(goalHeading);
        drive.move(new XYPair(), MathUtils.constrainDouble(power, -maxPower, maxPower));
    }

    @Override
    public boolean isFinished(){
        return headingModule.isOnTarget();
    }
}
