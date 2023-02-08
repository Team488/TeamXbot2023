package competition.subsystems.drive.commands;

import javax.inject.Inject;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.VisionSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import xbot.common.command.DelayViaSupplierCommand;
import xbot.common.math.XYPair;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.drive.control_logic.HeadingModule;
import xbot.common.subsystems.drive.control_logic.HeadingModule.HeadingModuleFactory;

public class RotateToVisionTargetCommand extends SequentialCommandGroup {

    private final Logger log = LogManager.getLogger(this.getName());

    private final PoseSubsystem pose;
    private final DriveSubsystem drive;
    private final VisionSubsystem vision;
    private final HeadingModule headingModule;

    private final DoubleProperty minWaitTime;

    @Inject
    public RotateToVisionTargetCommand(PropertyFactory pf, HeadingModuleFactory headingModuleFactory,
            PoseSubsystem pose, DriveSubsystem drive, VisionSubsystem vision) {
        pf.setPrefix(this.getName());

        this.pose = pose;
        this.drive = drive;
        this.vision = vision;

        this.headingModule = headingModuleFactory.create(drive.getRotateToHeadingPid());

        this.minWaitTime = pf.createPersistentProperty("Minimum wait time", 1.0);

        WaitUntilCommand waitForLock = new WaitUntilCommand(() -> this.vision.getFixAcquired());
        DelayViaSupplierCommand lockTimeout = new DelayViaSupplierCommand(() -> this.minWaitTime.get());

        addCommands(new ParallelRaceGroup(waitForLock, lockTimeout));

        ParallelRaceGroup setTargetsGroup = new ParallelRaceGroup(
                new WaitCommand(3),
                new RunCommand(() -> drive.move(new XYPair(0, 0), calculateDrivePower(this.vision.getBearingToHub())),
                        drive),
                new WaitUntilCommand(() -> headingModule.isOnTarget()));

        ConditionalCommand conditionalSetTarget = new ConditionalCommand(
                setTargetsGroup,
                new InstantCommand(() -> log.warn("No fix on target")),
                () -> this.vision.getFixAcquired());

        addCommands(conditionalSetTarget);

        addCommands(new InstantCommand(() -> this.drive.stop()));
    }

    private double calculateDrivePower(double relativeTarget) {
        return headingModule
                .calculateHeadingPower(pose.getCurrentHeading().plus(Rotation2d.fromDegrees(relativeTarget)));
    }
}
