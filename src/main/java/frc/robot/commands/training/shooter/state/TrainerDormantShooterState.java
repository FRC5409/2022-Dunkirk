package frc.robot.commands.training.shooter.state;

import org.jetbrains.annotations.NotNull;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.base.Property;
import frc.robot.base.command.StateCommandBase;
import frc.robot.base.shooter.ShooterState;
import frc.robot.base.shooter.odometry.SimpleShooterOdometry;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.shooter.ShooterTurret;
import frc.robot.training.TrainerDashboard;
import frc.robot.training.TrainerContext;
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
public class TrainerDormantShooterState extends StateCommandBase {
    private final Property<SimpleShooterOdometry> sharedOdometry;
    private final Property<ShooterState> shooterState;
    private final Property<Boolean> shooterTriggerDebounce;

    private final TrainerDashboard dashboard;
    private final TrainerContext context;

    private final Trigger shooterTrigger;

    private final ShooterTurret turret;
    private final Limelight limelight;

    private SimpleShooterOdometry odometry;
    private boolean debounce;

    public TrainerDormantShooterState(
        TrainerDashboard dashboard,
        TrainerContext context,
        ShooterTurret turret,
        Limelight limelight,
        Trigger shooterTrigger,
        Property<SimpleShooterOdometry> sharedOdometry,
        Property<ShooterState> shooterState,
        Property<Boolean> shooterTriggerDebounce
    ) {
        this.shooterTriggerDebounce = shooterTriggerDebounce;
        this.shooterTrigger = shooterTrigger;
        this.sharedOdometry = sharedOdometry;
        this.shooterState = shooterState;
        this.dashboard = dashboard;
        this.limelight = limelight;
        this.context = context;
        this.turret = turret;

        addRequirements(limelight, turret);
    }

    @Override
    public void initialize() {        
        if (!Toggleable.isEnabled(limelight, turret))
            throw new RuntimeException("Cannot operate shooter when requirements are not enabled.");

        // Initialize odometry
        debounce = shooterTriggerDebounce.get();
        odometry = sharedOdometry.get();

        shooterState.set(ShooterState.kDormant);
    }

    @Override
    public void execute() {
        if (odometry.isLost()) {
            next("frc.robot.shooter.sweep");
        } else {
            context.setDistance(odometry.getDistance());
            if (shooterTrigger.get() && !debounce) {
                next("arm");
            } else if (debounce) {
                debounce = false;
            }
        }

        dashboard.update();
    }
    
    @Override
    public void end(boolean interrupted) {
        shooterTriggerDebounce.set(false);
    }

    @Override
    public @NotNull String getStateName() {
        return "dormant";
    }
}
