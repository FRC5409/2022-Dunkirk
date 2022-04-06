package frc.robot.commands.shooter.state.operate;

import org.jetbrains.annotations.NotNull;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.base.Property;
import frc.robot.base.command.InterruptType;
import frc.robot.base.command.StateBase;
import frc.robot.base.shooter.ShooterState;
import frc.robot.base.shooter.odometry.DriveShooterOdometry;

// TODO update doc
public class DormantShooterState extends StateBase {
    private final Property<DriveShooterOdometry> sharedOdometry;
    private final Property<ShooterState> shooterState;
    private final Property<Boolean> shooterTriggerDebounce;
    private final Property<Double> driveSpeed;

    private final Trigger shooterTrigger;
    
    private DriveShooterOdometry odometry;
    private boolean debounce;

    public DormantShooterState(
        Trigger shooterTrigger,
        Property<DriveShooterOdometry> sharedOdometry,
        Property<ShooterState> shooterState,
        Property<Boolean> shooterTriggerDebounce,
        Property<Double> driveSpeed
    ) {
        this.shooterTriggerDebounce = shooterTriggerDebounce;
        this.shooterTrigger = shooterTrigger;
        this.sharedOdometry = sharedOdometry;
        this.shooterState = shooterState;
        this.driveSpeed = driveSpeed;
    }

    @Override
    public void initialize() {
        odometry = sharedOdometry.get();

        debounce = shooterTriggerDebounce.get();
        shooterState.set(ShooterState.kDormant);
        driveSpeed.set(1.0);
    }

    @Override
    public void execute() {
        if (odometry.isLost()) {
            next("frc.robot.shooter.sweep");
        } else {
            if (shooterTrigger.get() && !debounce) {
                next("arm");
            } else if (debounce) {
                debounce = false;
            }
        }
    }
    
    @Override
    public void end(InterruptType interrupt) {
        shooterTriggerDebounce.set(false);
    }

    @Override
    public @NotNull String getName() {
        return "dormant";
    }
}
