package frc.robot.commands.shooter.state;

import org.jetbrains.annotations.NotNull;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.base.Property;
import frc.robot.base.command.StateCommandBase;
import frc.robot.base.shooter.ShooterConfiguration;
import frc.robot.base.shooter.ShooterExecutionModel;
import frc.robot.base.shooter.DriveByShooterOdometry;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.TargetType;
import frc.robot.subsystems.shooter.ShooterFlywheel;
import frc.robot.subsystems.shooter.ShooterTurret;

import frc.robot.utils.Toggleable;

import frc.robot.Constants;

// TODO update doc

/**
 * Experimental
 */
public class ActiveOperateRunShooterState extends StateCommandBase {
    private final Property<DriveByShooterOdometry> sharedOdometry;
    private final Property<ShooterConfiguration> configuration;
    private final Property<Integer> offset;

    private final ShooterFlywheel flywheel;
    private final ShooterTurret turret;
    private final DriveTrain drivetrain;
    private final Limelight limelight;
    private final Indexer indexer;
    
    private ShooterExecutionModel model;
    private DriveByShooterOdometry odometry;
    
    public ActiveOperateRunShooterState(
        ShooterFlywheel flywheel,
        ShooterTurret turret,
        DriveTrain drivetrain,
        Limelight limelight,
        Indexer indexer,
        Property<ShooterConfiguration> configuration,
        Property<DriveByShooterOdometry> sharedOdometry,
        Property<Integer> offset
    ) {
        this.sharedOdometry = sharedOdometry;
        this.configuration = configuration;
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.flywheel = flywheel;
        this.indexer = indexer;
        this.turret = turret;
        this.offset = offset;

        addRequirements(limelight, turret, flywheel, indexer);
    }

    @Override
    public void initialize() {
        if (!Toggleable.isEnabled(limelight, turret, flywheel))
            throw new RuntimeException("Cannot arm shooter when requirements are not enabled.");
        else if (sharedOdometry.get() == null)
            throw new RuntimeException("Cannot operate shooter when odometry is not initialized");

        if (!indexer.isEnabled())
            indexer.enable();
        
        model = configuration.get().getExecutionModel();
        odometry = sharedOdometry.get();
        
        // Run feeder and indexer
        flywheel.spinFeeder(Constants.Shooter.FEEDER_VELOCITY);
        indexer.setSpeed(Constants.Shooter.INDEXER_SPEED);
    }

    @Override
    public void execute() {
        if (model == null)
            return;

        // Update odometry target
        if (limelight.getTargetType() == TargetType.kHub) {
            odometry.update(
                limelight.getTargetPosition(),
                drivetrain.getEncoderVelocity(),
                turret.getRotation()
            );
        }
            
        // Set flywheel to estimated veloctity
        double velocity = model.calculate(odometry.getDistance());
        flywheel.setVelocity(velocity + offset.get() + odometry.getFlywheelOffset());

        // Continue aligning shooter
        turret.setRotationTarget(
            MathUtil.interpolate(
                turret.getRotationTarget(),
                odometry.getTurretOffset() - odometry.getRotation(),
                SmartDashboard.getNumber("Rotation Smoothing", 0)
            )
        );

        if (Constants.kConfig.DEBUG) {
            SmartDashboard.putNumber("Flywheel - Velocity Prediction", velocity);
            SmartDashboard.putNumber("Flywheel - Active Velocity", flywheel.getVelocity());
            SmartDashboard.putNumber("Flywheel - Velocity Offset", offset.get());

            SmartDashboard.putData("Robot Odometry", odometry);
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
        return false;
    }

    @Override
    public @NotNull String getStateName() {
        return "frc.robot.shooter:operate:run";
    }
}
