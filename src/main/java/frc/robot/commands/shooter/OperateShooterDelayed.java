package frc.robot.commands.shooter;

import frc.robot.base.Property;
import frc.robot.base.ProxyStateCommandGroup;
import frc.robot.base.shooter.ShooterConfiguration;
import frc.robot.base.shooter.SweepDirection;
import frc.robot.commands.shooter.state.DelayedAlignShooterState;
import frc.robot.commands.shooter.state.OperateArmShooterState;
import frc.robot.commands.shooter.state.OperateRunShooterState;
import frc.robot.commands.shooter.state.SearchShooterState;
import frc.robot.commands.shooter.state.SweepShooterState;

import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.shooter.ShooterFlywheel;
import frc.robot.subsystems.shooter.ShooterTurret;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This is a StateCommandGroup commmand that handles all the states for the shooter. 
 * The states are the Search state, Sweep state, Align state, and the OperateShooterStaged state. 
 * The default state is the search state. 
 * 
 * @author Keith Davies
 */
public final class OperateShooterDelayed extends ProxyStateCommandGroup {
    public OperateShooterDelayed(
        Limelight limelight,
        ShooterTurret turret,
        ShooterFlywheel flywheel,
        Indexer indexer,
        Property<SweepDirection> direction,
        Property<ShooterConfiguration> configuration,
        Property<Integer> offset,
        Property<Boolean> armed,
        Trigger trigger
    ) {

        addCommands(
            new SearchShooterState(limelight, true),
            new SweepShooterState(limelight, turret, direction),
            new DelayedAlignShooterState(trigger, limelight, turret),
            new OperateArmShooterState(limelight, turret, flywheel, configuration, offset, armed),
            new OperateRunShooterState(limelight, turret, flywheel, indexer, configuration, offset)
        ); 

        setDefaultState("frc.robot.shooter:search");
    }
}