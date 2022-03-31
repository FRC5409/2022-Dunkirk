package frc.robot.commands.shooter.state;

import org.jetbrains.annotations.NotNull;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.base.Model4;
import frc.robot.base.Property;
import frc.robot.base.command.StateCommandBase;
import frc.robot.base.indexer.IndexerArmedState;
import frc.robot.base.shooter.ShooterConfiguration;
import frc.robot.base.shooter.ShooterState;
import frc.robot.base.shooter.odometry.DriveShooterOdometry;
import frc.robot.commands.shooter.RotateTurret;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.TargetType;
import frc.robot.subsystems.shooter.ShooterFlywheel;
import frc.robot.subsystems.shooter.ShooterTurret;
import frc.robot.subsystems.shooter.ShooterTurret.ReferenceType;
import frc.robot.utils.Toggleable;
import frc.robot.utils.Vector2;
import frc.robot.Constants;

// TODO update doc

/**
 * Experimental
 */
public class ActiveOperateRunShooterState extends StateCommandBase {
    private final Property<IndexerArmedState> indexerArmedState;
    private final Property<ShooterState> shooterState;
    
    private final Property<DriveShooterOdometry> sharedOdometry;
    private final Property<ShooterConfiguration> configuration;
    private final Property<Integer> offset;

    private final PIDController sharedController;

    private final ShooterFlywheel flywheel;
    private final ShooterTurret turret;
    private final DriveTrain drivetrain;
    private final Limelight limelight;
    private final Indexer indexer;
    private final Trigger trigger;
    
    private DriveShooterOdometry odometry;
    private boolean active;
    private boolean done;
    private Model4 model;

    private Property<Double> driveSpeed;
    private Property<Boolean> buttonDebounce;
    
    public ActiveOperateRunShooterState(
        ShooterFlywheel flywheel,
        ShooterTurret turret,
        DriveTrain drivetrain,
        Limelight limelight,
        Indexer indexer,
        Trigger trigger,
        Property<Boolean> buttonDebounce,
        Property<ShooterState> shooterState,
        Property<IndexerArmedState> indexerArmedState,
        Property<Double> driveSpeed,
        Property<ShooterConfiguration> configuration,
        Property<DriveShooterOdometry> sharedOdometry,
        PIDController sharedController,
        Property<Integer> offset
    ) {
        this.buttonDebounce = buttonDebounce;
        this.indexerArmedState = indexerArmedState;
        this.shooterState = shooterState;

        this.sharedController = sharedController;
        this.sharedOdometry = sharedOdometry;
        this.configuration = configuration;
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.flywheel = flywheel;
        this.trigger = trigger;
        this.indexer = indexer;
        this.turret = turret;
        this.offset = offset;

        this.driveSpeed = driveSpeed;
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
        active = false;
        done = false;

        shooterState.set(ShooterState.kActive);
    }

    @Override
    public void execute() {
        if (model == null)
            return;

        sharedController.setP(SmartDashboard.getNumber("Shooter P", 0.0));
        sharedController.setI(SmartDashboard.getNumber("Shooter I", 0.0));
        sharedController.setD(SmartDashboard.getNumber("Shooter D", 0.0));

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
        
        if (odometry.getTarget() != null && !odometry.isLost()) {
            double offset = Math.toRadians(odometry.getTurretOffset());
            
            Vector2 kNextDirection = new Vector2(Math.cos(offset), Math.sin(offset)).unit();
            Vector2 kActiveDirection = odometry.getVisionDirection();

            double relativeRotation = Math.atan2(
                kActiveDirection.x * kNextDirection.y - kActiveDirection.y * kNextDirection.x,
                kActiveDirection.x * kNextDirection.x + kActiveDirection.y * kNextDirection.y
            );

            SmartDashboard.putNumber("Relative Target Rotation", relativeRotation);

            double output = sharedController.calculate(relativeRotation);
            SmartDashboard.putNumber("Direction output", output);

            if (Math.abs(output) > SmartDashboard.getNumber("Shooter Output Thresh", 0))
                turret.setReference(output, ReferenceType.kOutput);
            else
                turret.setReference(0, ReferenceType.kOutput);

            updateDashboard(kNextDirection);
        }

        flywheel.setVelocity(velocity);

        if (flywheel.isTargetReached() && flywheel.feederReachedTarget() && !active) {
            indexer.setSpeed(Constants.Shooter.INDEXER_SPEED);
            active = true;
        }

        SmartDashboard.putNumber("Flywheel - Velocity Prediction", velocity);
        if (Constants.kConfig.DEBUG) {
            SmartDashboard.putNumber("Flywheel - Active Velocity", flywheel.getVelocity());
            SmartDashboard.putNumber("Flywheel - Velocity Offset", offset.get());

            SmartDashboard.putData("Robot Odometry", odometry);
        }

        if (!trigger.get()) {
            next("frc.robot.shooter:align");
            done = true;
            buttonDebounce.set(true);
        } else if (indexerArmedState.get() != IndexerArmedState.kArmed) {
            next("frc.robot.shooter:operate");
            done = true;
        }

    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted || getNextState() == null) {
            limelight.disable();
            flywheel.disable();
            indexer.disable();
            turret.disable();
        } else if (getNextState().equals("frc.robot.shooter:operate")) {
            flywheel.stopFeeder();
            indexer.disable();
            System.out.println("Stopped feeder");
        } else if (getNextState().equals("frc.robot.shooter:align")) {
            indexer.disable();
            flywheel.disable();
        }
        driveSpeed.set(1.0);
    }

    @Override
    public boolean isFinished() {
        return done;
    }

    @Override
    public @NotNull String getStateName() {
        return "frc.robot.shooter:operate:run";
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
