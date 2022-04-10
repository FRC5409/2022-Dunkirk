package frc.robot.commands.indexer.experimental;

import frc.robot.base.Joystick;
import frc.robot.base.Property;
import frc.robot.base.command.ProxyStateGroupCommand;
import frc.robot.base.indexer.IndexerState;
import frc.robot.base.shooter.ShooterState;
import frc.robot.commands.indexer.experimental.state.IndexerIntakeState;
import frc.robot.commands.indexer.experimental.state.intake.IndexerIndexState;
import frc.robot.commands.indexer.experimental.state.intake.IndexerPipeState;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;

public class IndexerIntakeActive2 extends ProxyStateGroupCommand {
    private final Property<IndexerState> indexerState;

    public IndexerIntakeActive2(
        Indexer indexer,
        Intake intake,
        Joystick joystick,
        Property<ShooterState> shooterState,
        Property<IndexerState> indexerState
    ) {
        this.indexerState = indexerState;

        addStates(
            new IndexerIntakeState(intake, shooterState)
                .addStates(
                    new IndexerPipeState(intake, shooterState, indexerState),
                    new IndexerIndexState(indexer, intake, joystick, indexerState)
                )
        );

        setDefaultState("frc.robot.indexer.intake");
    }

    @Override
    public void end(boolean interrupted) {
        indexerState.set(IndexerState.kOff);
    }
}
