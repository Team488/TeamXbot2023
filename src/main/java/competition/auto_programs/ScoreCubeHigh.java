
package competition.auto_programs;

import competition.subsystems.arm.UnifiedArmSubsystem;
import competition.subsystems.arm.commands.SimpleSafeArmRouterCommand;
import competition.subsystems.claw.CloseClawCommand;
import competition.subsystems.claw.OpenClawCommand;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.drive.commands.SwerveToPointCommand;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import xbot.common.math.XYPair;

import javax.inject.Inject;
import javax.inject.Provider;

public class ScoreCubeHigh extends SequentialCommandGroup {
    @Inject
    ScoreCubeHigh(Provider<OpenClawCommand> openClawProvider,
                  Provider<CloseClawCommand> closeClawProvider,
                  Provider<SimpleSafeArmRouterCommand> setArmPosProvider,
                  Provider<SwerveToPointCommand> swerveToPointProvider,
                  UnifiedArmSubsystem arm,
                  DriveSubsystem drive,
                  PoseSubsystem pose) {

        // Set scoring mode to Cube
        this.addCommands(arm.createSetGamePieceModeCommand(UnifiedArmSubsystem.GamePieceMode.Cube));

        //move arm to high
        var moveArmToHigh = setArmPosProvider.get();
        moveArmToHigh.setTarget(UnifiedArmSubsystem.KeyArmPosition.HighGoal, UnifiedArmSubsystem.RobotFacing.Backward);

        this.addCommands(moveArmToHigh);

        //open claw and close claw
        var openClaw = openClawProvider.get();
        var closeClaw = closeClawProvider.get();

        var openClawThenClose = new ParallelRaceGroup(
                new ParallelCommandGroup(openClaw, new WaitCommand(1), closeClaw),
                new WaitCommand(5));
        this.addCommands(openClawThenClose);

        //retract arm
        var retractArm = setArmPosProvider.get();
        retractArm.setTarget(UnifiedArmSubsystem.KeyArmPosition.FullyRetracted, UnifiedArmSubsystem.RobotFacing.Backward);
        this.addCommands(retractArm);

        //drive out of community zone
        var moveOutOfCommunity = swerveToPointProvider.get();
        moveOutOfCommunity.setFieldRelativeMotion();
        moveOutOfCommunity.setMaxPower(0.5);

        // Our target has to change if we are on red vs blue
        moveOutOfCommunity.setTargetSupplier(
                () -> {
                    var wpiXY = AutoLandmarks.convertBlueToRedIfNeeded(
                            AutoLandmarks.blueNorthOfChargingStationOutsideCommunity
                    ).getTranslation();
                    return new XYPair(wpiXY.getX(), wpiXY.getY());
                },
                () -> pose.rotateAngleBasedOnAlliance(Rotation2d.fromDegrees(0)).getDegrees()
        );

        var driveOut = new ParallelRaceGroup(
                moveOutOfCommunity,
                new WaitCommand(3));
        this.addCommands(driveOut);

        //stop robot
        this.addCommands(new InstantCommand(() -> drive.stop()));
    }
}
