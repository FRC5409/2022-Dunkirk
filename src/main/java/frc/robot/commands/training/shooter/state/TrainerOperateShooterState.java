package frc.robot.commands.training.shooter.state;

import org.jetbrains.annotations.NotNull;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.base.Property;
import frc.robot.base.command.StateCommandBase;
import frc.robot.base.shooter.odometry.ShooterExecutionModel;
import frc.robot.base.shooter.odometry.SimpleShooterOdometry;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.TargetType;
import frc.robot.subsystems.shooter.ShooterFlywheel;
import frc.robot.subsystems.shooter.ShooterTurret;
import frc.robot.training.TrainerDashboard;
import frc.robot.training.TrainerContext;
import frc.robot.utils.Toggleable;
import frc.robot.utils.Vector2;

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
public class TrainerOperateShooterState extends StateCommandBase {
    private final Property<SimpleShooterOdometry> sharedOdometry;
    private final Property<Boolean> armed;

    private final TrainerDashboard dashboard;
    private final TrainerContext context;

    private final ShooterFlywheel flywheel;
    private final ShooterTurret turret;
    private final Limelight limelight;
    private final Indexer indexer;

    private SimpleShooterOdometry odometry;
    private ShooterExecutionModel model;
    private boolean active;
    
    public TrainerOperateShooterState(
        TrainerDashboard dashboard,
        TrainerContext context,
        ShooterFlywheel flywheel,
        ShooterTurret turret,
        Limelight limelight,
        Indexer indexer,
        Property<SimpleShooterOdometry> sharedOdometry,
        Property<Boolean> armed
    ) {
        this.sharedOdometry = sharedOdometry;
        this.dashboard = dashboard;
        this.limelight = limelight;
        this.flywheel = flywheel;
        this.indexer = indexer;
        this.context = context;
        this.turret = turret;
        this.armed = armed;

        addRequirements(limelight, turret, flywheel, indexer);
    }

    @Override
    public void initialize() {
        if (!Toggleable.isEnabled(limelight, turret))
            throw new RuntimeException("Cannot operate shooter when requirements are not enabled.");

        if (!flywheel.isEnabled())
            flywheel.enable();

        if (!indexer.isEnabled())
            indexer.enable();

        // Initialize odometry
        odometry = sharedOdometry.get();
        model = context.getExecutionModel();

        // Spin feeder
        flywheel.spinFeeder(Constants.Shooter.FEEDER_VELOCITY);

        active = false;
    }

    @Override
    public void execute() {
        // Update odometry target
        if (limelight.getTargetType() == TargetType.kHub)
            odometry.update(limelight.getTargetPosition());


        if (odometry.hasTarget()) {
            Vector2 target = odometry.getTarget();

            double velocity = context.getSetpoint().getTarget();

            // Set flywheel to estimated veloctity
            flywheel.setVelocity(velocity);

            // Continue aligning shooter
            if (Math.abs(target.x) > Constants.Vision.ALIGNMENT_THRESHOLD)
                turret.setRotationTarget(turret.getRotation() + target.x* Constants.Vision.ROTATION_P);
                
            SmartDashboard.putNumber("Aligninment Offset", target.x);
        }

        if (!active && turret.isTargetReached() && flywheel.isTargetReached() && flywheel.feederReachedTarget() && armed.get()) {
            indexer.setSpeed(Constants.Shooter.INDEXER_SPEED);
            active = true;
        }
            
        context.setDistance(odometry.getDistance());

        SmartDashboard.putNumber("Active Velocity", flywheel.getVelocity());
        SmartDashboard.putData("Shooter Odometry", odometry);

        dashboard.update();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted || getNextState() == null) {
            limelight.disable();
            flywheel.disable();
            indexer.disable();
            turret.disable();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public @NotNull String getStateName() {
        return "frc.robot.shooter:operate";
    }
}
