package competition.subsystems.drive;

import com.google.inject.Inject;
import com.google.inject.Singleton;

import org.apache.log4j.Logger;

import competition.electrical_contract.ElectricalContract;
import xbot.common.controls.actuators.XCANTalon;
import xbot.common.injection.wpi_factories.CommonLibFactory;
import xbot.common.math.PIDFactory;
import xbot.common.math.PIDManager;
import xbot.common.math.XYPair;
import xbot.common.properties.XPropertyManager;
import xbot.common.subsystems.drive.BaseDriveSubsystem;

@Singleton
public class DriveSubsystem extends BaseDriveSubsystem {
    private static Logger log = Logger.getLogger(DriveSubsystem.class);
    
    ElectricalContract contract;
    
    public final XCANTalon leftLeader;
    public final XCANTalon rightLeader;

    private final PIDManager positionPid;
    private final PIDManager rotationPid;

    private double scalingFactorFromTicksToInches = 1.0 / 256.0;

    @Inject
    public DriveSubsystem(CommonLibFactory factory, XPropertyManager propManager, ElectricalContract contract, PIDFactory pf) {
        log.info("Creating DriveSubsystem");

        this.leftLeader = factory.createCANTalon(contract.getLeftLeader());
        this.rightLeader = factory.createCANTalon(contract.getRightLeader());

        positionPid = pf.createPIDManager(getPrefix() + "PositionPID");
        rotationPid = pf.createPIDManager(getPrefix() + "RotationPID");
    }

    public void tankDrive(double leftPower, double rightPower) {
        this.leftLeader.simpleSet(leftPower);
        this.rightLeader.simpleSet(rightPower);
    }

    @Override
    public PIDManager getPositionalPid() {
        return positionPid;
    }

    @Override
    public PIDManager getRotateToHeadingPid() {
        return rotationPid;
    }

    @Override
    public PIDManager getRotateDecayPid() {
        return null;
    }

    @Override
    public void move(XYPair translate, double rotate) {
        double y = translate.y;

        double left = y - rotate;
        double right = y + rotate;

        this.leftLeader.simpleSet(left);
        this.rightLeader.simpleSet(right);
    }

    @Override
    public double getLeftTotalDistance() {
        return leftLeader.getSelectedSensorPosition(0) * scalingFactorFromTicksToInches;
    }

    @Override
    public double getRightTotalDistance() {
        return rightLeader.getSelectedSensorPosition(0) * scalingFactorFromTicksToInches;
    }

    @Override
    public double getTransverseDistance() {
        return 0;
    }
}
