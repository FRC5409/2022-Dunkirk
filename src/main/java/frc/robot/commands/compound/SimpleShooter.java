package frc.robot.commands.compound;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.base.CommandProperty;
import frc.robot.base.Property;
import frc.robot.base.command.CommandLock;
import frc.robot.base.command.ProxySequentialCommandGroup;
import frc.robot.base.command.SynchronizedCommand;
import frc.robot.base.command.WrapperCommand;
import frc.robot.base.indexer.IndexerState;
import frc.robot.base.shooter.ShooterConfiguration;
import frc.robot.base.shooter.ShooterMode;
import frc.robot.base.shooter.ShooterState;
import frc.robot.commands.shooter.ConfigureShooter;
import frc.robot.commands.shooter.RotateTurret;
import frc.robot.commands.shooter.RunShooter;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.shooter.ShooterFlywheel;
import frc.robot.subsystems.shooter.ShooterTurret;

public class SimpleShooter extends WrapperCommand {
    public SimpleShooter(
        ShooterFlywheel flywheel,
        ShooterTurret turret,
        Limelight limelight,
        Indexer indexer,
        CommandLock shooterLock,
        CommandProperty<Boolean> shooterActiveToggle,
        CommandProperty<Boolean> shooterRunning,
        CommandProperty<IndexerState> indexerState,
        CommandProperty<ShooterState> shooterState,
        Property<ShooterConfiguration> shooterConfiguration,
        ShooterMode shooterMode,
        double flywheelSpeed,
        double indexerSpeed
    ) {
        super(
            new SynchronizedCommand(
                shooterLock,
                new ProxySequentialCommandGroup(
                    // Set shooter running to false
                    shooterRunning.configureTo(false),

                    // Set shooter active toggle to false
                    shooterActiveToggle.configureTo(false),
                    
                    // Wait for indexer to become armed
                    indexerState.equalTo(IndexerState.kArmed),

                    // Configure shooter to target mode
                    new ConfigureShooter(turret, limelight, shooterConfiguration, shooterMode),
                    
                    new ParallelCommandGroup(
                        // Rotate turret to zero position
                        new RotateTurret(turret, 0),

                        // Run shooter at specified flywheel and indexer speed
                        new RunShooter(flywheel, indexer, shooterState, flywheelSpeed, indexerSpeed)
                    )
                )
            )
        );
    }
}
