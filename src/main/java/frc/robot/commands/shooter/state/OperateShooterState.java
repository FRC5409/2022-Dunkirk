package frc.robot.commands.shooter.state;

import org.jetbrains.annotations.NotNull;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.base.Property;
import frc.robot.base.StateCommandBase;
import frc.robot.base.shooter.ShooterConfiguration;
import frc.robot.base.shooter.ShooterModel;

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
    private final Property<ShooterConfiguration> configuration;
    private final Property<Integer> offset;
    private final ShooterFlywheel flywheel;
    private final ShooterTurret turret;
    private final Limelight limelight;
    private final Indexer indexer;
    
    private ShooterModel model;
    private boolean active;
    private Vector2 target;
    
    public OperateShooterState(
        Limelight limelight,
        ShooterTurret turret,
        ShooterFlywheel flywheel,
        Indexer indexer,
        Property<ShooterConfiguration> configuration,
        Property<Integer> offset
    ) {
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

        flywheel.spinFeeder(Constants.Shooter.FEEDER_VELOCITY);

        target = limelight.getTargetPosition();
        model = configuration.get().getModel();

        active = false;
    }

    @Override
    public void execute() {
        if (model == null)
            return;

        if (limelight.getTargetType() == TargetType.kHub)
            target = limelight.getTargetPosition();
            
        double distance = model.distance(target.y);
        double velocity = model.calculate(distance);



        // Set flywheel to estimated veloctity
        flywheel.setVelocity(velocity + offset.get());

        // Continue aligning shooter
        if (Math.abs(target.x) > Constants.Vision.ALIGNMENT_THRESHOLD)
            turret.setRotationTarget(turret.getRotation() + target.x * Constants.Vision.ROTATION_P);

        if (!active && turret.isTargetReached() && flywheel.isTargetReached() && flywheel.feederReachedTarget()) {
            if (Constants.kConfig.DEBUG) {
                System.out.println("Target Reached");
            }

            indexer.spinIndexer(Constants.Shooter.INDEXER_VELOCITY);
            active = true;
        }

        if (Constants.kConfig.DEBUG) {
            SmartDashboard.putNumber("Velocity Prediction", velocity);
            SmartDashboard.putNumber("Active Velocity", flywheel.getVelocity());
            
            SmartDashboard.putNumber("Distance Prediction (ft)", distance);
            SmartDashboard.putNumber("Aligninment Offset", target.x);
            SmartDashboard.putNumber("Velocity Offset", offset.get());
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
