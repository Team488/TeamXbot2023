package competition.subsystems.drive.commands;

import javax.inject.Inject;

import competition.operator_interface.OperatorInterface;
import competition.subsystems.drive.DriveSubsystem;
import xbot.common.command.BaseCommand;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

public class VelocityDriveWithJoysticksCommand extends BaseCommand {

    private final DriveSubsystem drive;
    private final OperatorInterface oi;
    private final DoubleProperty maxVelocity;

    @Inject
    public VelocityDriveWithJoysticksCommand(DriveSubsystem drive, OperatorInterface oi, PropertyFactory pf) {
        this.drive = drive;
        this.oi = oi;
        pf.setPrefix(this.getName());
        this.maxVelocity = pf.createPersistentProperty("Max Velocity", 0.5);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        double input = oi.driverGamepad.getLeftStickY();
        drive.setVelocityMaintainerXTarget(input * maxVelocity.get());
    }
    
}
