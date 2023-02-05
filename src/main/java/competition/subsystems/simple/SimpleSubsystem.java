package competition.subsystems.simple;

import xbot.common.command.BaseSubsystem;
import xbot.common.controls.actuators.XCANSparkMax;
import xbot.common.injection.electrical_contract.DeviceInfo;

import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public class SimpleSubsystem extends BaseSubsystem {

    XCANSparkMax sparkMax;

    @Inject
    SimpleSubsystem(XCANSparkMax.XCANSparkMaxFactory sparkMaxFactory) {
        //sparkMax = sparkMaxFactory.create(
        //        new DeviceInfo(35), this.getPrefix(), "SimpleMotor");
    }

    public void setPower(double power) {
        //sparkMax.set(power);
    }
}
