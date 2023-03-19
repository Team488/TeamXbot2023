package competition.auto_programs;

import competition.auto_programs.support.AutonomousOracle;
import competition.commandgroups.MoveCollectedGamepieceToArmCommandGroup;
import competition.commandgroups.ScoreCubeHighCommandGroup;
import competition.commandgroups.ScoreGamepieceCommandGroupFactory;
import competition.subsystems.arm.UnifiedArmSubsystem;
import competition.subsystems.arm.commands.SimpleXZRouterCommand;
import competition.subsystems.collector.CollectorSubsystem;
import competition.subsystems.collector.commands.EjectCollectorCommand;
import competition.subsystems.drive.commands.AutoBalanceCommand;
import competition.subsystems.drive.commands.BrakeCommand;
import competition.subsystems.drive.commands.SwerveSimpleTrajectoryCommand;
import competition.subsystems.drive.commands.VelocityMaintainerCommand;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import javax.inject.Inject;
import javax.inject.Provider;

/**
 * A command group that conditionally runs many commands depending on driver selection.
 */
public class ParameterizedAutonomousProgram extends SequentialCommandGroup {

    @Inject
    public ParameterizedAutonomousProgram(
            AutonomousOracle oracle,
            PoseSubsystem pose,
            UnifiedArmSubsystem arms,
            CollectorSubsystem collector,
            Provider<EjectCollectorCommand> ejectCollectorCommandProvider,
            ScoreGamepieceCommandGroupFactory scoreGamepieceCommandGroupFactory,
            Provider<SimpleXZRouterCommand> setArmPosProvider,
            Provider<ScoreCubeHighCommandGroup> scoreCubeHighProvider,
            Provider<SwerveSimpleTrajectoryCommand> swerveSimpleTrajectoryCommandProvider,
            AutoBalanceCommand autoBalance,
            VelocityMaintainerCommand velocityMaintainer,
            Provider<MoveCollectedGamepieceToArmCommandGroup> moveCollectedGamepieceToArmCommandGroupProvider,
            BrakeCommand brake) {

        // ----------------------------
        // Set initial position
        // ----------------------------

        double defaultVelocity = 80;

        // TODO: If we trust the april tags, we should have a branch here where we don't force the initial position (and
        // potentially the initial heading, if the april tags pose estimation gets really good).
        var setInitialPosition = new InstantCommand(
                () -> {
                    pose.setCurrentPoseInMeters(oracle.getInitialPoseInMeters());
                }
        );

        this.addCommands(setInitialPosition);

        var setInitialGamepiece = new InstantCommand(
                () -> {
                    arms.setGamePieceMode(oracle.getInitialGamePiece());
                }
        );
        this.addCommands(setInitialGamepiece);

        // ----------------------------
        // Score the initial game piece, either by ejecting or by using the arm.
        // ----------------------------

        // TODO: may want to use different collector powers or durations for the different game pieces
        var scoreViaEjecting = ejectCollectorCommandProvider.get().withTimeout(1).andThen(new InstantCommand(collector::stop));;

        var scoreViaArm = scoreGamepieceCommandGroupFactory.create(UnifiedArmSubsystem.KeyArmPosition.HighGoal, true);

        // OnTrue, OnFalse, and the condition. This pattern will repeat throughout this class, as there are a lot of forks
        // in this autonomous program.
        var scoreSomehow = new ConditionalCommand(
                scoreViaEjecting,
                scoreViaArm,
                () -> oracle.getInitialScoringMode() == AutonomousOracle.ScoringMode.Eject);

        this.addCommands(scoreSomehow);

        // ----------------------------
        // Optionally drive somewhere interesting (outside for mobility, or towards a game piece)
        // ----------------------------

        var drivePhaseOne = swerveSimpleTrajectoryCommandProvider.get();
        drivePhaseOne.setMaxPower(0.75);
        drivePhaseOne.setMaxTurningPower(0.33);
        drivePhaseOne.setKeyPointsProvider(oracle::getTrajectoryForDrivePhaseOne);
        drivePhaseOne.setEnableConstantVelocity(true);
        drivePhaseOne.setConstantVelocity(defaultVelocity);

        var drivePhaseOneOrNot = new ConditionalCommand(
                drivePhaseOne,
                new InstantCommand(),
                oracle::getEnableDrivePhaseOne
        );

        this.addCommands(drivePhaseOneOrNot);

        // ----------------------------
        // Optionally acquire a game piece
        // ----------------------------

        var collect = collector.getCollectThenRetractCommand().withTimeout(1.0);

        var collectOrNot = new ConditionalCommand(
                collect,
                new InstantCommand(),
                oracle::getEnableAcquireGamePiece
        );

        this.addCommands(collectOrNot);

        // ----------------------------
        // Optionally drive back to scoring or other useful position
        // ----------------------------

        var driveForScoring = swerveSimpleTrajectoryCommandProvider.get();
        driveForScoring.setMaxPower(0.75);
        driveForScoring.setMaxTurningPower(0.33);
        driveForScoring.setKeyPointsProvider(oracle::getTrajectoryForScoring);
        driveForScoring.setEnableConstantVelocity(true);
        driveForScoring.setConstantVelocity(defaultVelocity);

        // If we're planning on scoring using the arm, we should move the game piece to the claw.

        var moveGamePieceToArm = moveCollectedGamepieceToArmCommandGroupProvider.get();

        var moveGamePieceToArmOrNot = new ConditionalCommand(
                moveGamePieceToArm,
                new InstantCommand(),
                () -> oracle.getSecondScoringMode() != AutonomousOracle.ScoringMode.Eject
        );

        var driveForScoringOrNot = new ConditionalCommand(
                driveForScoring.alongWith(moveGamePieceToArmOrNot),
                new InstantCommand(),
                oracle::getEnableMoveToScore
        );

        this.addCommands(driveForScoringOrNot);

        // ----------------------------
        // Optionally score the game piece
        // ----------------------------

        var scoreViaEjectingAgain = ejectCollectorCommandProvider.get().withTimeout(1).andThen(new InstantCommand(collector::stop));
        var scoreHighAgain = scoreGamepieceCommandGroupFactory.create(UnifiedArmSubsystem.KeyArmPosition.HighGoal, true);

        var scoreAgain = new ConditionalCommand(
                scoreViaEjectingAgain,
                scoreHighAgain,
                () -> oracle.getSecondScoringMode() == AutonomousOracle.ScoringMode.Eject);

        var scoreAgainOrNot = new ConditionalCommand(
                scoreAgain,
                new InstantCommand(),
                oracle::getEnableSecondScore
        );

        this.addCommands(scoreAgainOrNot);

        // ----------------------------
        // Optionally balance
        // ----------------------------

        var driveToBalance = swerveSimpleTrajectoryCommandProvider.get();
        driveToBalance.setMaxPower(0.75); // TODO: hopefully tune this up to go faster, but too fast makes me nervous.
        driveToBalance.setMaxTurningPower(0.33);
        driveToBalance.setKeyPointsProvider(oracle::getTrajectoryForBalance);
        driveToBalance.setEnableConstantVelocity(true);
        driveToBalance.setConstantVelocity(defaultVelocity);


        var balance = new ParallelRaceGroup(
                autoBalance,
                velocityMaintainer);
                //new WaitUntilCommand(() -> DriverStation.getMatchTime() < 1.0)
        //);

        var balanceOrNot = new ConditionalCommand(
                driveToBalance,
                new InstantCommand(),
                oracle::getEnableBalance
        );

        this.addCommands(balanceOrNot);
        this.addCommands(balance);
    }
}
