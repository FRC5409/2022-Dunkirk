package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.base.Property;
import frc.robot.base.shooter.ShooterState;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.shooter.ShooterFlywheel;

/**
 * This command runs the turret flywheel at a speed
 * porportional to the distance of the turret from the
 * shooting target and operates the indexer once the rpm
 * reaches it's setpoint.
 * 
 * @author Keith Davies
 */
public final class RunShooter extends CommandBase {
    private final Property<ShooterState> shooterState;
    private final ShooterFlywheel flywheel;
    private final Indexer indexer;

    private final double target;
    private final double indexerTarget;

    private boolean active;

    public RunShooter(
        ShooterFlywheel flywheel,
        Indexer indexer,
        Property<ShooterState> shooterState,
        double target,
        double indexerTarget
    ) {
        this.indexerTarget = indexerTarget; 
        this.shooterState = shooterState;
        this.flywheel = flywheel;
        this.indexer = indexer;
        this.target = target;
    }
    
    public RunShooter(
        ShooterFlywheel flywheel,
        Indexer indexer,
        Property<ShooterState> shooterState,
        double target
    ) {
        this(flywheel, indexer, shooterState, target, 1);
    }

    @Override
    public void initialize() {
        flywheel.enable();
        indexer.enable();

        flywheel.setVelocity(target);
        flywheel.spinFeeder(Constants.Shooter.FEEDER_VELOCITY);

        shooterState.set(ShooterState.kRun);

        active = false;
    }

    @Override
    public void execute() {     
        if (!active && flywheel.isTargetReached() && flywheel.feederReachedTarget()) {
            indexer.setSpeed(indexerTarget);
            active = true;
        }
    }


    @Override
    public void end(boolean interrupted) {
        flywheel.disable();
        indexer.disable();
        shooterState.set(ShooterState.kOff);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}