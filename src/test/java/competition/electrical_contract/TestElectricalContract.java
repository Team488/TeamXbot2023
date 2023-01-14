package competition.electrical_contract;

import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public class TestElectricalContract extends CompetitionContract {

    private boolean canCodersReady = true;

    @Inject
    public TestElectricalContract() {}

    @Override
    public boolean isDriveReady() {
        return true;
    }

    @Override
    public boolean areCanCodersReady() {
        return canCodersReady;
    }

    public void setCanCodersReady(boolean areReady) {
        canCodersReady = areReady;
    }
}
