package frc.robot.commands.shooter;

import frc.robot.Constants;
import frc.robot.base.Property;
import frc.robot.base.command.TimedCommand;
import frc.robot.base.indexer.IndexerState;
import frc.robot.subsystems.Indexer;

public class PrimeShooter extends TimedCommand {
    private final Property<IndexerState> indexerArmedState;
    private final Indexer indexer;


    public PrimeShooter(Indexer indexer, Property<IndexerState> indexerArmedState) {
        this.indexer = indexer;
        this.indexerArmedState = indexerArmedState;

        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        super.initialize();
        indexer.enable();
        indexer.setSpeed(-0.5);
        indexerArmedState.set(IndexerState.kPrime);
    }


    @Override
    public void end(boolean interrupted) {
        if (!interrupted)
            indexerArmedState.set(IndexerState.kArmed);

        indexer.disable();
    }

    @Override
    public boolean isFinished() {
        return getElapsedTime() > Constants.Shooter.ARMING_TIME;
    }
}
