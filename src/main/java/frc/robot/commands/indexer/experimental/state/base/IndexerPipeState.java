package frc.robot.commands.indexer.experimental.state.base;

import org.jetbrains.annotations.NotNull;

import frc.robot.base.Property;
import frc.robot.base.command.InterruptType;
import frc.robot.base.command.StateBase;
import frc.robot.base.indexer.IndexerState;
import frc.robot.base.shooter.ShooterState;
import frc.robot.subsystems.Intake;

public class IndexerPipeState extends StateBase {
    private final Property<IndexerState> indexerState;
    private final Property<ShooterState> shooterState;
    private final Intake intake;

    public IndexerPipeState(Intake intake, Property<ShooterState> shooterState, Property<IndexerState> indexerState) {
        this.indexerState = indexerState;
        this.shooterState = shooterState;
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.intakeOn(0.4);
    }

    @Override
    public void execute() {
        if (!shooterState.isEqual(ShooterState.kRun))
            next("index");
    }

    @Override
    public void end(InterruptType interrupt) {
        if (interrupt == InterruptType.kCancel) {
            intake.intakeOn(0);
        }
    }
    
    @Override
    public @NotNull String getName() {
        return "pipe";
    }
    
}
