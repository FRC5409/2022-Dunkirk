package frc.robot.commands.indexer.experimental.state;

import org.jetbrains.annotations.NotNull;

import frc.robot.base.command.InterruptType;
import frc.robot.base.command.StateBase;
import frc.robot.subsystems.Intake;

public class IndexerBaseState extends StateBase {
    private final Intake intake;

    public IndexerBaseState(Intake intake) {
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.solenoidsDown();
    }

    @Override
    public void end(InterruptType interrupt) {
        intake.solenoidsUp();
    }
    
    @Override
    public @NotNull String getName() {
        return "frc.robot.indexer";
    }
}
