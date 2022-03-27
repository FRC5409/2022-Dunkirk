package frc.robot.commands.shooter.state;

import org.jetbrains.annotations.NotNull;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.base.Property;
import frc.robot.base.command.StateCommandBase;
import frc.robot.base.shooter.ShooterConfiguration;
import frc.robot.base.shooter.odometry.ShooterExecutionModel;
import frc.robot.base.shooter.odometry.SimpleShooterOdometry;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.TargetType;
import frc.robot.subsystems.shooter.ShooterFlywheel;
import frc.robot.subsystems.shooter.ShooterTurret;

import frc.robot.utils.Toggleable;
import frc.robot.utils.Vector2;

import frc.robot.Constants;

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
public class OperateShooterState extends StateCommandBase {
    private final Property<SimpleShooterOdometry> sharedOdometry;
    private final Property<ShooterConfiguration> configuration;
    private final Property<Integer> offset;
    private final ShooterFlywheel flywheel;
    private final ShooterTurret turret;
    private final Limelight limelight;
    private final Indexer indexer;
    
    private SimpleShooterOdometry odometry;
    private ShooterExecutionModel model;
    private boolean active;
    
    public OperateShooterState(
        ShooterFlywheel flywheel,
        ShooterTurret turret,
        Limelight limelight,
        Indexer indexer,
        Property<SimpleShooterOdometry> sharedOdometry,
        Property<ShooterConfiguration> configuration,
        Property<Integer> offset
    ) {
        this.sharedOdometry = sharedOdometry;
        this.configuration = configuration;
        this.limelight = limelight;
        this.flywheel = flywheel;
        this.indexer = indexer;
        this.turret = turret;
        this.offset = offset;

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

        // Obtain shooter configuration
        ShooterConfiguration config = configuration.get();

        // Initialize odometry
        model = config.getExecutionModel();
        odometry = sharedOdometry.get();

        // pre spin feeder
        flywheel.spinFeeder(Constants.Shooter.FEEDER_VELOCITY);

        active = false;
    }

    @Override
    public void execute() {
        if (model == null)
            return;

        // Update odometry target
        if (limelight.getTargetType() == TargetType.kHub)
            odometry.update(limelight.getTargetPosition());
        
        if (odometry.hasTarget()) {
            Vector2 target = odometry.getTarget();
            
            // Set flywheel to estimated veloctity
            double velocity = model.calculate(odometry.getDistance());
            flywheel.setVelocity(velocity + offset.get());

            // Continue aligning shooter
            if (Math.abs(target.x) > Constants.Vision.ALIGNMENT_THRESHOLD)
                turret.setRotationTarget(turret.getRotation() + target.x * Constants.Vision.ROTATION_P);

            // Query targets
            if (!active && turret.isTargetReached() && flywheel.isTargetReached() && flywheel.feederReachedTarget()) {
                if (Constants.kConfig.DEBUG) {
                    System.out.println("Target Reached");
                }

                indexer.setSpeed(Constants.Shooter.INDEXER_SPEED);
                active = true;
            }

            if (Constants.kConfig.DEBUG) {
                SmartDashboard.putNumber("Velocity Prediction", velocity);
                SmartDashboard.putNumber("Active Velocity", flywheel.getVelocity());
                
                SmartDashboard.putNumber("Distance Prediction (ft)", odometry.getDistance());
                SmartDashboard.putNumber("Aligninment Offset", target.x);
                SmartDashboard.putNumber("Velocity Offset", offset.get());
            }
        }
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
        return false; //!(limelight.hasTarget() && limelight.getTargetType() == TargetType.kHub);
    }

    @Override
    public @NotNull String getStateName() {
        return "frc.robot.shooter:operate";
    }
}
