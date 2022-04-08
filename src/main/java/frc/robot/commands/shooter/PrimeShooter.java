package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.base.Property;
import frc.robot.base.indexer.IndexerArmedState;
import frc.robot.subsystems.Indexer;

public class PrimeShooter extends CommandBase {
    private final Property<IndexerArmedState> indexerArmedState;
    private final Indexer indexer;

    public PrimeShooter(Indexer indexer, Property<IndexerArmedState> indexerArmedState) {
        this.indexer = indexer;
        this.indexerArmedState = indexerArmedState;
    }

    @Override
    public void initialize() {
        indexer.enable();
        indexer.setSpeed(-0.5);
        indexerArmedState.set(IndexerArmedState.kActive);
    }


    @Override
    public void end(boolean interrupted) {
        indexer.disable();
        indexerArmedState.set(IndexerArmedState.kArmed);
        System.out.println("Shooter armed interrupted - " + interrupted);
    }
}
