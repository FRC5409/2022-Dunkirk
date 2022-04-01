package frc.robot.commands.shooter.v2.state;

import org.jetbrains.annotations.NotNull;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.base.Property;
import frc.robot.base.ValueProperty;
import frc.robot.base.command.StateCommandBase;
import frc.robot.base.indexer.IndexerArmedState;
import frc.robot.base.shooter.ShooterConditionType;
import frc.robot.base.shooter.ShooterConditions;
import frc.robot.base.shooter.ShooterConfiguration;
import frc.robot.base.shooter.ShooterState;
import frc.robot.base.shooter.odometry.DriveShooterOdometry;
import frc.robot.base.shooter.odometry.ShooterTrackingModel;
import frc.robot.commands.shooter.v2.state.operate.ArmShooterState;
import frc.robot.commands.shooter.v2.state.operate.DormantShooterState;
import frc.robot.commands.shooter.v2.state.operate.RunShooterState;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.TargetType;
import frc.robot.subsystems.shooter.ShooterFlywheel;
import frc.robot.subsystems.shooter.ShooterTurret;
import frc.robot.subsystems.shooter.ShooterTurret.ReferenceType;
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
public class OperateShooterState extends StateCommandBase {
    private final Property<DriveShooterOdometry> sharedOdometry;
    private final Property<ShooterConfiguration> shooterConfiguration;
    private final Property<IndexerArmedState> indexerArmedState;
    
    private final ShooterConditions shooterConditions;
    private final PIDController turretController;

    private final ShooterTurret turret;
    private final DriveTrain drivetrain;
    private final Limelight limelight;
    
    private ShooterTrackingModel trackingModel;
    private DriveShooterOdometry odometry;
    
    public OperateShooterState(
        ShooterFlywheel flywheel,
        ShooterTurret turret,
        DriveTrain drivetrain,
        Limelight limelight,
        Indexer indexer,
        Trigger shooterTrigger,
        PIDController turretController,
        ShooterConditions shooterConditions,
        Property<ShooterConfiguration> shooterConfiguration,
        Property<DriveShooterOdometry> sharedOdometry,
        Property<IndexerArmedState> indexerArmedState,
        Property<ShooterState> shooterState,
        Property<Double> shooterOffset,
        Property<Double> driveSpeed
    ) {
        this.shooterConfiguration = shooterConfiguration;
        this.indexerArmedState = indexerArmedState;
        this.shooterConditions = shooterConditions;
        this.turretController = turretController;
        this.sharedOdometry = sharedOdometry;
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.turret = turret;

        Property<Boolean> shooterTriggerDebounce = new ValueProperty<>(false);

        addStates(
            new DormantShooterState(shooterConditions, shooterTrigger, sharedOdometry, 
                shooterState, shooterTriggerDebounce),
            
            new ArmShooterState(flywheel, shooterConditions, shooterTrigger, shooterConfiguration, 
                sharedOdometry, indexerArmedState, shooterState, shooterTriggerDebounce, 
                shooterOffset, driveSpeed),

            new RunShooterState(flywheel, indexer, shooterTrigger, shooterConfiguration, sharedOdometry,
                indexerArmedState, shooterState, shooterTriggerDebounce, 
                shooterOffset, driveSpeed)
        );

        addRequirements(limelight, turret);
    }

    @Override
    public void initialize() {
        if (!Toggleable.isEnabled(limelight))
            throw new RuntimeException("Cannot operate shooter when requirements are not enabled.");

        if (!limelight.isEnabled())
            limelight.enable();

        // Obtain shooter configuration
        ShooterConfiguration config = shooterConfiguration.get();

        // Initialize odometry
        odometry = sharedOdometry.get();
        trackingModel = config.getTrackingModel();

        next("dormant");
    }

    @Override
    public void execute() {
        // Update odometry target
        if (limelight.getTargetType() == TargetType.kHub)
            odometry.update(limelight.getTargetPosition());
        
        // Update odometry target
        if (limelight.getTargetType() == TargetType.kHub) {
            odometry.update(
                limelight.getTargetPosition(),
                drivetrain.getEncoderVelocity(),
                turret.getRotation()
            );
        }

        if (odometry.getTarget() != null) {
            double offset = Math.toRadians(odometry.getTurretOffset());
            
            Vector2 kNextDirection = new Vector2(Math.cos(offset), Math.sin(offset)).unit();
            Vector2 kActiveDirection = odometry.getVisionDirection();

            double relativeRotation = Math.atan2(
                kActiveDirection.x * kNextDirection.y - kActiveDirection.y * kNextDirection.x,
                kActiveDirection.x * kNextDirection.x + kActiveDirection.y * kNextDirection.y
            );

            double output = turretController.calculate(relativeRotation);
            if (Math.abs(output) > trackingModel.kMinOutput)
                turret.setReference(output, ReferenceType.kOutput);
            else
                turret.setReference(0, ReferenceType.kOutput);

            updateDashboard(kNextDirection);
        }
        
        if (turretController.atSetpoint())
            shooterConditions.addCondition(ShooterConditionType.kTurretReached);
        else
            shooterConditions.removeCondition(ShooterConditionType.kTurretReached);
    
        if (indexerArmedState.isEqual(IndexerArmedState.kArmed))
            shooterConditions.addCondition(ShooterConditionType.kIndexerArmed);
        else
            shooterConditions.removeCondition(ShooterConditionType.kIndexerArmed);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted || getNextState() == null) {
            limelight.disable();
            turret.disable();
        }
    }

    @Override
    public boolean isFinished() {
        return isDormant();
    }

    @Override
    public @NotNull String getStateName() {
        return "frc.robot.shooter.operate";
    }

    public void updateDashboard(Vector2 kNextDirection) {
        SmartDashboard.putNumber("Turret Rotation", turret.getRotation());
        SmartDashboard.putNumber("Odometry Rotation", odometry.getRotation());
        SmartDashboard.putNumber("Odometry Distance", odometry.getDistance());
        SmartDashboard.putNumber("Odometry Speed", odometry.getSpeed());
        SmartDashboard.putString("Odometry Target", odometry.getTarget().toString());
        SmartDashboard.putString("Odometry Vision Direction", odometry.getVisionDirection().toString());
        SmartDashboard.putString("Odometry Direction", odometry.getDirection().toString());
        SmartDashboard.putString("Odometry Next Direction", kNextDirection.toString());
        SmartDashboard.putString("Odometry Velocity", odometry.getVelocity().toString());
        SmartDashboard.putNumber("Odometry Turret Offset", odometry.getTurretOffset());
        SmartDashboard.putNumber("Odometry Flywheel Offset", odometry.getFlywheelOffset());
    }
}
