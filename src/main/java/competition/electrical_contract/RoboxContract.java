package competition.electrical_contract;

import javax.inject.Inject;

public class RoboxContract extends CompetitionContract{

    @Inject
    public RoboxContract() {}

    @Override
    public boolean areCanCodersReady() {
        return false;
    }

    @Override
    public boolean isCollectorReady() {
        return false;
    }

    @Override
    public boolean isDriveReady() {
        return false;
    }

    @Override
    public boolean isLowerArmEncoderReady() {
        return false;
    }

    @Override
    public boolean isLowerArmReady() {
        return false;
    }

    @Override
    public boolean isUpperArmEncoderReady() {
        return false;
    }

    @Override
    public boolean isUpperArmReady() {
        return false;
    }
}
