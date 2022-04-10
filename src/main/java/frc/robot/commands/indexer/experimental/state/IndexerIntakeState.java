package frc.robot.commands.indexer.experimental.state;

import org.jetbrains.annotations.NotNull;

import frc.robot.base.Property;
import frc.robot.base.command.InterruptType;
import frc.robot.base.command.StateBase;
import frc.robot.base.shooter.ShooterState;
import frc.robot.subsystems.Intake;

public class IndexerIntakeState extends StateBase {
    private final Property<ShooterState> shooterState;

    private final Intake intake;

    public IndexerIntakeState(Intake intake, Property<ShooterState> shooterState) {
        this.shooterState = shooterState;
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
        return "frc.robot.indexer.intake";
    }
}
