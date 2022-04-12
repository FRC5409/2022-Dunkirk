package frc.robot.commands.shooter;

import frc.robot.Constants;
import frc.robot.base.Property;
import frc.robot.base.command.TimedCommand;
import frc.robot.base.indexer.IndexerArmedState;
import frc.robot.subsystems.Indexer;

public class PrimeShooter extends TimedCommand {
    private final Property<IndexerArmedState> indexerArmedState;
    private final Indexer indexer;


    public PrimeShooter(Indexer indexer, Property<IndexerArmedState> indexerArmedState) {
        this.indexer = indexer;
        this.indexerArmedState = indexerArmedState;

        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        super.initialize();
        indexer.enable();
        indexer.setSpeed(-0.5);
        indexerArmedState.set(IndexerArmedState.kActive);
    }


    @Override
    public void end(boolean interrupted) {
        if (!interrupted)
            indexerArmedState.set(IndexerArmedState.kArmed);

        indexer.disable();
    }

    @Override
    public boolean isFinished() {
        return getElapsedTime() > Constants.Shooter.ARMING_TIME;
    }
}
