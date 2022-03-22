package frc.robot.base.shooter.odometry;

import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.utils.Vector2;
import frc.robot.utils.Vector3;

/**
 * Experimental shooter position relative odometry.
 */
public class SimpleShooterOdometry extends OdometryBase {
    protected double kLastDistance;
    protected Vector2 kLastTarget;

    public SimpleShooterOdometry(ShooterOdometryModel model) {
        super(model);
        
        kLastDistance = 0;
        kLastTarget = new Vector2();
    }

    /**
     * 
     * @param target   The observed vision target
     */
    public void update(Vector2 target) {
        Vector3 observerVector = calculateTargetProjection(target);

        kLastDistance = model.kHeight / Math.tan(Math.asin(observerVector.z));
        if (!Double.isFinite(kLastDistance))
            kLastDistance = 0;

        kLastTarget = new Vector2(target);
    }

    @Override
    public void reset() {
        kLastDistance = 0;
        kLastTarget = new Vector2();
    }

    public double getDistance() {
        return kLastDistance;
    }

    public Vector2 getTarget() {
        return new Vector2(kLastTarget);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Target", () -> getTarget().toString(), null);
        builder.addDoubleProperty("Distance", this::getDistance, null);
    }
}
