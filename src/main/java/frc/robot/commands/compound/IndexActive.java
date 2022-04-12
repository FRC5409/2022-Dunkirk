package frc.robot.commands.compound;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.base.CommandProperty;
import frc.robot.base.Joystick;
import frc.robot.base.command.CancelCommand;
import frc.robot.base.command.CommandLock;
import frc.robot.base.command.ProxySequentialCommandGroup;
import frc.robot.base.command.SynchronizedCommand;
import frc.robot.base.command.WrapperCommand;
import frc.robot.base.indexer.IndexerState;
import frc.robot.base.shooter.ShooterState;
import frc.robot.commands.indexer.IndexerIntakeActive;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class IndexActive extends WrapperCommand {
    public IndexActive(
        Indexer indexer,
        Intake intake,
        Joystick joystick,
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

                    // Run indexer intake active
                    new IndexerIntakeActive(indexer, intake, joystick)
                )
            )
        );
    }
}
