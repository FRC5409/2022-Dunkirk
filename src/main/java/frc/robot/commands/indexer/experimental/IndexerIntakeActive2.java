package frc.robot.commands.indexer.experimental;

import frc.robot.base.Joystick;
import frc.robot.base.Property;
import frc.robot.base.command.ProxyStateGroupCommand;
import frc.robot.base.indexer.IndexerState;
import frc.robot.base.shooter.ShooterState;
import frc.robot.commands.indexer.experimental.state.IndexerBaseState;
import frc.robot.commands.indexer.experimental.state.base.IndexerIndexState;
import frc.robot.commands.indexer.experimental.state.base.IndexerPipeState;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;

public class IndexerIntakeActive2 extends ProxyStateGroupCommand {
    private final Property<ShooterState> shooterState;
    private final Property<IndexerState> indexerState;

    public IndexerIntakeActive2(
        Indexer indexer,
        Intake intake,
        Joystick joystick,
        Property<ShooterState> shooterState,
        Property<IndexerState> indexerState
    ) {
        this.shooterState = shooterState;
        this.indexerState = indexerState;

        addStates(
            new IndexerBaseState(intake)
                .addStates(
                    new IndexerPipeState(intake, shooterState, indexerState),
                    new IndexerIndexState(indexer, intake, joystick, indexerState)
                )
        );
    }

    @Override
    public void initialize() {
        if (shooterState.isEqual(ShooterState.kRun))
            setActiveState("frc.robot.indexer:pipe");
        else
            setActiveState("frc.robot.indexer:index");

        super.initialize();
    }
}
