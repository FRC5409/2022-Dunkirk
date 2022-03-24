package frc.robot.base.shooter;

import org.jetbrains.annotations.Nullable;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.utils.Vector2;

public class ShooterTarget {
    private Vector2 target;
    private double last;

    public ShooterTarget() {
        target = null;
        last = Timer.getFPGATimestamp();
    }

    public void update(Vector2 target) {
        this.target = target;
        last = Timer.getFPGATimestamp();
    }

    public void reset() {
        target = null;
        last = Timer.getFPGATimestamp();
    }

    public boolean lost() {
        return (Timer.getFPGATimestamp() - last) > Constants.Shooter.TARGET_LOST_TIME;
    }

    @Nullable
    public Vector2 getTarget() {
        return target;
    }

    public boolean hasTarget() {
        return target != null && !lost();
    }
}
