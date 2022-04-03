package frc.robot.commands.training.shooter.state;

import org.jetbrains.annotations.NotNull;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.base.Property;
import frc.robot.base.command.StateCommandBase;
import frc.robot.base.indexer.IndexerArmedState;
import frc.robot.base.shooter.ShooterState;
import frc.robot.base.shooter.odometry.SimpleShooterOdometry;
import frc.robot.base.training.TrainerContext;
import frc.robot.base.training.TrainerDashboard;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.TargetType;
import frc.robot.subsystems.shooter.ShooterFlywheel;
import frc.robot.subsystems.shooter.ShooterTurret;
import frc.robot.utils.Toggleable;

// TODO update doc

/**
 * <h3>Operates the turret in the "Shooting" state.</h>
 * 
 * <p>In this state, the turret runs it's flywheel at a speed 
 * porportional to the distance of the turret from the outer port
 * and aligns the turret's rotation axis to the target.
 * Once both systems have reached their respective targets,
 * the indexer triggers, feeding powercells into the turret.</p>
 */
public class TrainerRunShooterState extends StateCommandBase {
    private final Property<SimpleShooterOdometry> sharedOdometry;
    private final Property<IndexerArmedState> indexerArmedState;
    private final Property<Boolean> shooterTriggerDebounce;

    private final TrainerDashboard dashboard;
    private final TrainerContext context;

    private final Trigger shooterTrigger;
    
    private final ShooterFlywheel flywheel;
    private final Indexer indexer;

    private SimpleShooterOdometry odometry;
    private boolean active;
    
    public TrainerRunShooterState(
        TrainerDashboard dashboard,
        TrainerContext context,
        ShooterFlywheel flywheel,
        Indexer indexer,
        Trigger shooterTrigger,
        Property<SimpleShooterOdometry> sharedOdometry,
        Property<IndexerArmedState> indexerArmedState,
        Property<ShooterState> shooterState,
        Property<Boolean> shooterTriggerDebounce
    ) {
        this.shooterTriggerDebounce = shooterTriggerDebounce;
        this.indexerArmedState = indexerArmedState;
        this.shooterTrigger = shooterTrigger;
        this.sharedOdometry = sharedOdometry;
        this.dashboard = dashboard;
        this.flywheel = flywheel;
        this.indexer = indexer;
        this.context = context;

        addRequirements(flywheel, indexer);
    }

    @Override
    public void initialize() {
        if (!Toggleable.isEnabled(flywheel))
            throw new RuntimeException("Cannot operate shooter when requirements are not enabled.");

        if (!indexer.isEnabled())
            indexer.enable();

        // Initialize odometry
        odometry = sharedOdometry.get();

        // Spin feeder
        flywheel.spinFeeder(Constants.Shooter.FEEDER_VELOCITY);

        active = false;
    }

    @Override
    public void execute() {
        // Set flywheel to estimated veloctity
        if (odometry.hasTarget()) {
            flywheel.setVelocity(context.getSetpoint().getTarget());
            context.setDistance(odometry.getDistance());
        }
        if (flywheel.isTargetReached() && flywheel.feederReachedTarget() && !active) {
            indexer.setSpeed(Constants.Shooter.INDEXER_SPEED);
            active = true;
        }

        if (!shooterTrigger.get() || !indexerArmedState.isEqual(IndexerArmedState.kArmed)) {
            shooterTriggerDebounce.set(true);
            next("dormant");
        }

        dashboard.update();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted || getNextState() == null) {
            flywheel.disable();
            indexer.disable();
        } else {
            flywheel.stopFeeder();
            indexer.disable();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public @NotNull String getStateName() {
        return "run";
    }
}
