package competition.subsystems.drive.commands;

import javax.inject.Inject;

import competition.operator_interface.OperatorInterface;
import competition.subsystems.drive.DriveSubsystem;
import xbot.common.command.BaseCommand;
import xbot.common.math.MathUtils;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

public class PositionDriveWithJoysticksCommand extends BaseCommand {

    private final DriveSubsystem drive;
    private final OperatorInterface oi;
    private final DoubleProperty maxPositionalChange;

    @Inject
    public PositionDriveWithJoysticksCommand(DriveSubsystem drive, OperatorInterface oi, PropertyFactory pf) {
        this.drive = drive;
        this.oi = oi;
        pf.setPrefix(this.getName());
        this.maxPositionalChange = pf.createPersistentProperty("Max Positional Change", 0.1);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        double input = MathUtils.deadband(oi.driverGamepad.getLeftStickY(), oi.getDriverGamepadTypicalDeadband(),
                (a) -> a);
        double delta = input * maxPositionalChange.get();
        drive.setPositionMaintainerXTarget(drive.getPositionMaintainerXTarget() + delta);
    }
    
}
