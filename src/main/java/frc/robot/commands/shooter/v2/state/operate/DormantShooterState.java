package frc.robot.commands.shooter.v2.state.operate;

import org.jetbrains.annotations.NotNull;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.base.Property;
import frc.robot.base.command.StateCommandBase;
import frc.robot.base.shooter.ShooterConditions;
import frc.robot.base.shooter.ShooterState;
import frc.robot.base.shooter.odometry.DriveShooterOdometry;

// TODO update doc
public class DormantShooterState extends StateCommandBase {
    private final Property<DriveShooterOdometry> sharedOdometry;
    private final Property<ShooterState> shooterState;
    private final Property<Boolean> shooterTriggerDebounce;
    
    private final ShooterConditions shooterConditions;
    private final Trigger shooterTrigger;
    
    private DriveShooterOdometry odometry;
    private boolean debounce;

    public DormantShooterState(
        ShooterConditions shooterConditions,
        Trigger shooterTrigger,
        Property<DriveShooterOdometry> sharedOdometry,
        Property<ShooterState> shooterState,
        Property<Boolean> shooterTriggerDebounce
    ) {
        this.shooterTriggerDebounce = shooterTriggerDebounce;
        this.shooterConditions = shooterConditions;
        this.shooterTrigger = shooterTrigger;
        this.sharedOdometry = sharedOdometry;
        this.shooterState = shooterState;
    }

    @Override
    public void initialize() {
        odometry = sharedOdometry.get();

        debounce = shooterTriggerDebounce.get();
        shooterState.set(ShooterState.kDormant);
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
    public void end(boolean interrupted) {
        shooterTriggerDebounce.set(false);
    }

    @Override
    public @NotNull String getStateName() {
        return "dormant";
    }
}
