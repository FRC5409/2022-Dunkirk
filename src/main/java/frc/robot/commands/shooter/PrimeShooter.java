package frc.robot.commands.shooter;

import frc.robot.Constants;
import frc.robot.base.Property;
import frc.robot.base.command.TimedCommand;
import frc.robot.base.indexer.IndexerState;
import frc.robot.subsystems.Indexer;

public class PrimeShooter extends TimedCommand {
    private final Property<IndexerState> indexerState;
    private final Indexer indexer;


    public PrimeShooter(Indexer indexer, Property<IndexerState> indexerState) {
        this.indexer = indexer;
        this.indexerState = indexerState;

        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        super.initialize();

        indexer.enable();
        indexer.setSpeed(-0.5);
        
        indexerState.set(IndexerState.kActive);
    }


    @Override
    public void end(boolean interrupted) {
        if (!interrupted)
            indexerState.set(IndexerState.kArmed);

        indexer.disable();
    }

    @Override
    public boolean isFinished() {
        return getElapsedTime() > Constants.Shooter.ARMING_TIME;
    }
}
