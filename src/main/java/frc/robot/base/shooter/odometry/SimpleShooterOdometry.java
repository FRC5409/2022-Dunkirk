package frc.robot.base.shooter.odometry;

import org.jetbrains.annotations.Nullable;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.base.shooter.target.FilterBase;
import frc.robot.utils.Vector2;
import frc.robot.utils.Vector3;

/**
 * Experimental shooter position relative odometry.
 */
public class SimpleShooterOdometry extends ShooterOdometryBase {
    protected final FilterBase filter;

    protected Vector2 kLastTarget;
    protected double kLastDistance;
    protected double kLastUpdate;


    public SimpleShooterOdometry(ShooterOdometryModel model, FilterBase filter) {
        super(model);
        
        kLastDistance = 0;
        kLastTarget = null;
        kLastUpdate = 0;

        this.filter = filter;
    }

    /**
     * 
     * @param target   The observed vision target
     */
    public void update(Vector2 target) {
        Vector3 observerVector = calculateTargetProjection(filter.update(target));

        kLastDistance = model.kHeight / Math.tan(Math.asin(observerVector.z));
        if (!Double.isFinite(kLastDistance))
            kLastDistance = 0;

        kLastTarget = new Vector2(target);
        kLastUpdate = Timer.getFPGATimestamp();
    }

    @Override
    public void reset() {
        kLastDistance = 0;
        kLastTarget = null;
        kLastUpdate = Timer.getFPGATimestamp();

        filter.reset();
    }

    public double getDistance() {
        return kLastDistance;
    }

    @Nullable
    public Vector2 getTarget() {
        return new Vector2(kLastTarget);
    }

    public boolean hasTarget(){
        return kLastTarget != null;
    }

    public boolean isLost() {
        return (Timer.getFPGATimestamp() - kLastUpdate) > model.kkAcquistionTimeout;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Target", () -> String.valueOf(getTarget()), null);
        builder.addDoubleProperty("Distance", this::getDistance, null);
    }
}
