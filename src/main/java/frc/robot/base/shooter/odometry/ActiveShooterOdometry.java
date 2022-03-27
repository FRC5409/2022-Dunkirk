package frc.robot.base.shooter.odometry;

import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.base.shooter.target.FilterBase;
import frc.robot.utils.Vector2;
import frc.robot.utils.Vector3;

/**
 * Experimental shooter position relative odometry.
 */
public class ActiveShooterOdometry extends SimpleShooterOdometry {
    protected double  kLastSpeed;
    protected Vector2 kLastVelocity;
    protected double  kLastRotation;
    protected Vector2 kLastDirection;

    public ActiveShooterOdometry(ShooterOdometryModel model, FilterBase filter) {
        super(model, filter);
        
        kLastDirection = new Vector2();
        kLastVelocity = new Vector2();
        kLastDistance = 0;
        kLastRotation = 0;
        kLastTarget = new Vector2();
        kLastSpeed = 0;
    }

    @Override
    public void update(Vector2 target) {
        update(target, kLastSpeed, kLastRotation);
    }

    /**
     * 
     * @param target   The observed vision target
     * @param speed    The observed speed
     * @param rotation The observed view rotation
     */
    public void update(Vector2 target, double speed, double rotation) {
        Vector3 observerVector = calculateTargetProjection(target);

        rotation = Math.toRadians(rotation);

        double cx = Math.cos(-rotation);
        double cy = Math.sin(-rotation);

        kLastDirection = new Vector2(
            observerVector.x * cx - observerVector.y * cy,
            observerVector.x * cy + observerVector.y * cx
        ).unit();

        kLastDistance = safe(model.kHeight / Math.tan(Math.asin(observerVector.z)));
        kLastRotation = safe(Math.acos(kLastDirection.x)) * Math.signum(kLastDirection.y);
        kLastVelocity = kLastDirection.scale(speed);
        kLastTarget = new Vector2(target);
        kLastSpeed = speed;
    }

    @Override
    public void reset() {
        kLastDirection = new Vector2();
        kLastVelocity = new Vector2();
        kLastDistance = 0;
        kLastRotation = 0;
        kLastTarget = new Vector2();
        kLastSpeed = 0;
    }

    public Vector2 getVelocity() {
        return kLastVelocity;
    }

    public Vector2 getDirection() {
        return kLastDirection;
    }

    public double getRotation() {
        return kLastRotation;
    }

    public double getSpeed() {
        return kLastSpeed;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Velocity", () -> getVelocity().toString(), null);
        builder.addStringProperty("Direction", () -> getDirection().toString(), null);
        builder.addStringProperty("Target", () -> getTarget().toString(), null);
        builder.addDoubleProperty("Distance", this::getDistance, null);
        builder.addDoubleProperty("Speed", this::getSpeed, null);
        builder.addDoubleProperty("Rotation", () -> Math.toDegrees(this.getRotation()), null);
    }

    protected double safe(double x) {
        if (!Double.isFinite(x))
            return 0;
        return x;
    }
}
