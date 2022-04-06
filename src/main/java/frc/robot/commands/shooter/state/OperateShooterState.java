package frc.robot.commands.shooter.state;

import org.jetbrains.annotations.NotNull;

import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.base.Property;
import frc.robot.base.command.InterruptType;
import frc.robot.base.command.StateBase;
import frc.robot.base.indexer.IndexerArmedState;
import frc.robot.base.shooter.ShooterConditionType;
import frc.robot.base.shooter.ShooterConditions;
import frc.robot.base.shooter.ShooterConfiguration;
import frc.robot.base.shooter.TrackingController;
import frc.robot.base.shooter.odometry.DriveShooterOdometry;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.TargetType;
import frc.robot.subsystems.shooter.ShooterTurret;
import frc.robot.subsystems.shooter.ShooterTurret.ReferenceType;
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
public class OperateShooterState extends StateBase {
    private final Property<ShooterConfiguration> shooterConfiguration;
    private final Property<DriveShooterOdometry> sharedOdometry;
    private final Property<IndexerArmedState> indexerArmedState;
    
    private final ShooterConditions shooterConditions;
    
    private final ShooterTurret turret;
    private final DriveTrain drivetrain;
    private final Limelight limelight;
    
    private TrackingController trackingController;
    private DriveShooterOdometry odometry;
    
    public OperateShooterState(
        ShooterTurret turret,
        DriveTrain drivetrain,
        Limelight limelight,
        ShooterConditions shooterConditions,
        Property<ShooterConfiguration> shooterConfiguration,
        Property<DriveShooterOdometry> sharedOdometry,
        Property<IndexerArmedState> indexerArmedState
    ) {
        this.shooterConfiguration = shooterConfiguration;
        this.indexerArmedState = indexerArmedState;
        this.shooterConditions = shooterConditions;
        this.sharedOdometry = sharedOdometry;
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.turret = turret;

        addRequirements(limelight, turret);
    }

    @Override
    public void initialize() {
        if (!Toggleable.isEnabled(limelight))
            throw new RuntimeException("Cannot operate shooter when requirements are not enabled.");

        if (!turret.isEnabled())
            turret.enable();

        // Initialize odometry
        odometry = sharedOdometry.get();
        trackingController = new TrackingController(shooterConfiguration.get().getTrackingModel());
    }

    @Override
    public void execute() {
        // Update odometry target
        if (limelight.getTargetType() == TargetType.kHub) {
            odometry.update(
                limelight.getTargetPosition(),
                drivetrain.getEncoderVelocity(),
                turret.getRotation()
            );
        }

        if (odometry.getTarget() != null) {
            trackingController.update(
                odometry.getViewDirection(),
                odometry.getTurretOffset(),
                drivetrain.TurnRate(),
                odometry.getVelocity().y
            );

            turret.setReference(
                turret.getRotation() + trackingController.getOutput(), 
                ReferenceType.kRotation);
        }
        
        if (trackingController.isTargetReached())
            shooterConditions.addCondition(ShooterConditionType.kTurretReached);
        else
            shooterConditions.removeCondition(ShooterConditionType.kTurretReached);
    
        if (indexerArmedState.isEqual(IndexerArmedState.kArmed))
            shooterConditions.addCondition(ShooterConditionType.kIndexerArmed);
        else
            shooterConditions.removeCondition(ShooterConditionType.kIndexerArmed);
    }

    @Override
    public void end(InterruptType interrupt) {
        if (interrupt == InterruptType.kCancel || interrupt == InterruptType.kFinish) {
            turret.disable();
            limelight.disable();
        }
        
        odometry = null;
    }

    @Override
    public boolean isFinished() {
        return !isDormant();
    }

    @Override
    public @NotNull String getName() {
        return "frc.robot.shooter.operate";
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("Turret Rotation", turret::getRotation, null);
        builder.addDoubleProperty("Odometry Rotation",         () -> (hasOdometry() ? odometry.getRotation()                : 0), null);
        builder.addDoubleProperty("Odometry Distance",         () -> (hasOdometry() ? odometry.getDistance()                : 0), null);
        builder.addDoubleProperty("Odometry Speed",            () -> (hasOdometry() ? odometry.getSpeed()                   : 0), null);
        builder.addDoubleProperty("Odometry Turret Offset",    () -> (hasOdometry() ? odometry.getTurretOffset()            : 0), null);
        builder.addDoubleProperty("Odometry Flywheel Offset",  () -> (hasOdometry() ? odometry.getFlywheelOffset()          : 0), null);
        builder.addStringProperty("Odometry Target",           () -> (hasOdometry() ? odometry.getTarget().toString()        :  ""), null);
        builder.addStringProperty("Odometry Vision Direction", () -> (hasOdometry() ? odometry.getViewDirection().toString() :  ""), null);
        builder.addStringProperty("Odometry Direction",        () -> (hasOdometry() ? odometry.getDirection().toString()     :  ""), null);
        builder.addStringProperty("Odometry Velocity",         () -> (hasOdometry() ? odometry.getVelocity().toString()      :  ""), null);
    }

    private boolean hasOdometry() {
        return (odometry != null && odometry.getTarget() != null);
    }
}
