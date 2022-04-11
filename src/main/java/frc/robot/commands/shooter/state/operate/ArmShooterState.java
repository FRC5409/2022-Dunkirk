package frc.robot.commands.shooter.state.operate;

import org.jetbrains.annotations.NotNull;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.base.Property;
import frc.robot.base.command.InterruptType;
import frc.robot.base.command.StateBase;
import frc.robot.base.indexer.IndexerState;
import frc.robot.base.shooter.ShooterConditionType;
import frc.robot.base.shooter.ShooterConditions;
import frc.robot.base.shooter.ShooterModelProvider;
import frc.robot.base.shooter.ShooterState;
import frc.robot.base.shooter.odometry.DriveShooterOdometry;
import frc.robot.subsystems.shooter.ShooterFlywheel;
import frc.robot.utils.Equation;

// TODO update doc

/**
 * Experimental
 */
public class ArmShooterState extends StateBase {
    private final Property<DriveShooterOdometry> sharedOdometry;
    private final Property<ShooterState> shooterState;
    private final Property<Boolean> shooterTriggerDebounce;
    private final Property<Double> shooterOffset;
    private final Property<Double> driveSpeed;

    private final ShooterModelProvider shooterModelProvider;
    private final ShooterConditions shooterConditions;
    private final Trigger shooterTrigger;

    private final ShooterFlywheel flywheel;
    
    private DriveShooterOdometry odometry;
    private Equation executionModel;
    
    public ArmShooterState(
        ShooterFlywheel flywheel,
        ShooterConditions shooterConditions,
        Trigger shooterTrigger,
        ShooterModelProvider shooterModelProvider,
        Property<DriveShooterOdometry> sharedOdometry,
        Property<IndexerState> indexerState,
        Property<ShooterState> shooterState,
        Property<Boolean> shooterTriggerDebounce,
        Property<Double> shooterOffset,
        Property<Double> driveSpeed
    ) {
        this.shooterTriggerDebounce = shooterTriggerDebounce;
        this.shooterModelProvider = shooterModelProvider;
        this.shooterConditions = shooterConditions;
        this.sharedOdometry = sharedOdometry;
        this.shooterTrigger = shooterTrigger;
        this.shooterOffset = shooterOffset;
        this.shooterState = shooterState;
        this.driveSpeed = driveSpeed;
        this.flywheel = flywheel;

        addRequirements(flywheel);
    }

    @Override
    public void initialize() {
        if (!flywheel.isEnabled())
            flywheel.enable();
    
        executionModel = shooterModelProvider.getExecutionModel();
        odometry = sharedOdometry.get();
        
        // Check if previous state wasnt run
        if (shooterState.get() != ShooterState.kRun)
            driveSpeed.set(shooterModelProvider.getTrackingModel().kDriveSpeed);

        shooterState.set(ShooterState.kArm);
    }

    @Override
    public void execute() {
        if (executionModel == null)
            return;

        if (odometry.getTarget() != null && !odometry.isLost()) {
            flywheel.setVelocity(
                executionModel.calculate(odometry.getDistance()) + odometry.getFlywheelOffset() + shooterOffset.get()
            );
        }

        if (flywheel.isTargetReached())
            shooterConditions.addCondition(ShooterConditionType.kFlywheelReached);
        else
            shooterConditions.removeCondition(ShooterConditionType.kFlywheelReached);

        if (!shooterTrigger.get()) {
            shooterTriggerDebounce.set(true);
            next("dormant");
        } else if (shooterConditions.hasCondition(ShooterConditionType.kAll)) {
            next("run");
        }
    }

    @Override
    public void end(InterruptType interrupt) {
        if (interrupt == InterruptType.kCancel || getNextState() == null) {
            flywheel.disable();
            driveSpeed.set(1.0);
        } else if (getNextState().equals("dormant")) {
            flywheel.disable();
        }
    }

    @Override
    public @NotNull String getName() {
        return "arm";
    }
}
