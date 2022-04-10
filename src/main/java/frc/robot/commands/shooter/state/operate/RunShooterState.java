package frc.robot.commands.shooter.state.operate;

import org.jetbrains.annotations.NotNull;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.base.Property;
import frc.robot.base.command.InterruptType;
import frc.robot.base.command.StateBase;
import frc.robot.base.indexer.IndexerState;
import frc.robot.base.shooter.ShooterModelProvider;
import frc.robot.base.shooter.ShooterState;
import frc.robot.base.shooter.odometry.DriveShooterOdometry;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.shooter.ShooterFlywheel;
import frc.robot.utils.Equation;
import frc.robot.utils.Toggleable;
import frc.robot.Constants;

// TODO update doc

/**
 * Experimental
 */
public class RunShooterState extends StateBase {
    private final Property<DriveShooterOdometry> sharedOdometry;
    private final Property<IndexerState> indexerArmedState;
    private final Property<ShooterState> shooterState;
    private final Property<Boolean> shooterTriggerDebounce;
    private final Property<Double> shooterOffset;
    private final Property<Double> driveSpeed;

    private final ShooterModelProvider shooterModelProvider;
    private final Trigger shooterTrigger;

    private final ShooterFlywheel flywheel;
    private final Indexer indexer;
    
    private DriveShooterOdometry odometry;
    private Equation executionModel;
    private boolean active;
    
    public RunShooterState(
        ShooterFlywheel flywheel,
        Indexer indexer,
        Trigger shooterTrigger,
        ShooterModelProvider shooterModelProvider,
        Property<DriveShooterOdometry> sharedOdometry,
        Property<IndexerState> indexerArmedState,
        Property<ShooterState> shooterState,
        Property<Boolean> shooterTriggerDebounce,
        Property<Double> shooterOffset,
        Property<Double> driveSpeed
    ) {
        this.shooterTriggerDebounce = shooterTriggerDebounce;
        this.shooterModelProvider = shooterModelProvider;
        this.indexerArmedState = indexerArmedState;
        this.sharedOdometry = sharedOdometry;
        this.shooterTrigger = shooterTrigger;
        this.shooterOffset = shooterOffset;
        this.shooterState = shooterState;
        this.driveSpeed = driveSpeed;
        this.flywheel = flywheel;
        this.indexer = indexer;
    }

    @Override
    public void initialize() {
        if (!Toggleable.isEnabled(flywheel))
            throw new RuntimeException("Cannot run shooter when requirements are not enabled.");
        else if (sharedOdometry.get() == null)
            throw new RuntimeException("Cannot run shooter when odometry is not initialized");

        if (!indexer.isEnabled())
            indexer.enable();
    
        // Run feeder
        flywheel.setFeederOutput(Constants.Shooter.FEEDER_VELOCITY);
        
        shooterState.set(ShooterState.kRun);
        driveSpeed.set(shooterModelProvider.getTrackingModel().kDriveSpeed);
        
        executionModel = shooterModelProvider.getExecutionModel();
        odometry = sharedOdometry.get();
        active = false;

        System.out.println("RUN SHOOTER PURRRRRRR");
    }

    @Override
    public void execute() {
        if (executionModel == null)
            return;
        
        // Set flywheel to estimated veloctity
        if (odometry.hasTarget()) {
            flywheel.setVelocity(
                executionModel.calculate(odometry.getDistance()) + odometry.getFlywheelOffset() + shooterOffset.get()
            );
        }

        if (flywheel.isTargetReached() && flywheel.isFeederTargetReached() && !active) {
            indexer.setSpeed(Constants.Shooter.INDEXER_SPEED);
            active = true;
        } 

        if (!shooterTrigger.get()) {
            shooterTriggerDebounce.set(true);
            next("dormant");
        } else if (!indexerArmedState.isEqual(IndexerState.kArmed)) {
            next("arm");
        }

    }

    @Override
    public void end(InterruptType interrupt) {
        if (interrupt == InterruptType.kCancel || getNextState() == null || getNextState().equals("dormant")) {
            flywheel.disable();
        } else {
            flywheel.stopFeeder();
        }

        driveSpeed.set(1.0);
        indexer.disable();
    }

    @Override
    public @NotNull String getName() {
        return "run";
    }
}
