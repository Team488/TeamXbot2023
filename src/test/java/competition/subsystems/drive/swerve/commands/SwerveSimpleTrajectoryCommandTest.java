package competition.subsystems.drive.swerve.commands;

import competition.subsystems.drive.commands.SwerveSimpleTrajectoryCommand;
import competition.subsystems.drive.commands.XbotSwervePoint;
import org.junit.Test;

import java.util.ArrayList;

import static org.junit.Assert.assertEquals;

public class SwerveSimpleTrajectoryCommandTest extends BaseFullSwerveTest {

    SwerveSimpleTrajectoryCommand command;

    @Override
    public void setUp() {
        super.setUp();
        command = getInjectorComponent().swerveSimpleTrajectoryCommand();
    }

    @Test
    public void testVelocityNormalizedTrajectory() {
        ArrayList<XbotSwervePoint> points = new ArrayList<>();
        points.add(new XbotSwervePoint(0, 0, 0, 100));
        points.add(new XbotSwervePoint(0, 50, 0, 100));

        command.setEnableConstantVelocity(true);
        command.setConstantVelocity(1);
        command.setKeyPoints(points);

        command.initialize();
        var results = command.getResolvedKeyPoints();

        assertEquals(50, results.get(1).getSecondsForSegment(), 0.001);
    }
}
