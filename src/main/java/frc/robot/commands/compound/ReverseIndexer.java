package frc.robot.commands.compound;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.base.CommandProperty;
import frc.robot.base.Property;
import frc.robot.base.command.CancelCommand;
import frc.robot.base.command.CommandLock;
import frc.robot.base.command.ProxySequentialCommandGroup;
import frc.robot.base.command.SynchronizedCommand;
import frc.robot.base.command.WrapperCommand;
import frc.robot.base.indexer.IndexerState;
import frc.robot.base.shooter.ShooterState;
import frc.robot.commands.indexer.RunIndexer;
import frc.robot.subsystems.Indexer;

public class ReverseIndexer extends WrapperCommand {
    public ReverseIndexer(
        Indexer indexer,
        Command interruptTarget,
        CommandLock indexerLock,
        CommandProperty<IndexerState> indexerState,
        CommandProperty<ShooterState> shooterState
    ) {
        super(
            new SynchronizedCommand(
                indexerLock,
                new ProxySequentialCommandGroup(
                    // Cancel interrupt target command
                    new CancelCommand(interruptTarget),
                    
                    // Configure indexer state to active
                    indexerState.configureTo(IndexerState.kActive),

                    // Wait for shooter state to respect indexer state, and not run
                    shooterState.notEqualTo(ShooterState.kRun),

                    // Reverse Indexer
                    new RunIndexer(indexer, -0.5),
                    
                    // Configure indexer state to armed,
                    indexerState.configureTo(IndexerState.kArmed)
                )
            )
        );
    }
}
