package frc.robot.base.shooter.odometry;

import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.utils.Vector2;
import frc.robot.utils.Vector3;

/**
 * Experimental shooter position relative odometry.
 */
public class ActiveShooterOdometry extends OdometryBase {
    protected double  kLastSpeed;
    protected Vector2 kLastTarget;
    protected Vector2 kLastVelocity;
    protected double  kLastDistance;
    protected double  kLastRotation;
    protected Vector2 kLastDirection;

    public ActiveShooterOdometry(ShooterOdometryModel model) {
        super(model);
        
        kLastDistance = 0;
        kLastRotation = 0;
        kLastVelocity = new Vector2();
        kLastTarget = new Vector2();
        kLastSpeed = 0;
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

        kLastDirection = new Vector2(
            observerVector.x + Math.cos(rotation),
            observerVector.y + Math.sin(rotation)
        ).unit();

        kLastDistance = safe(model.kHeight / Math.tan(Math.asin(observerVector.z)));
        kLastRotation = safe(Math.acos(kLastDirection.x));
        kLastVelocity = kLastDirection.scale(speed);
        kLastTarget = new Vector2(target);
        kLastSpeed = speed;
    }

    @Override
    public void reset() {
        kLastDistance = 0;
        kLastRotation = 0;
        kLastDirection = new Vector2();
        kLastVelocity = new Vector2();
        kLastTarget = new Vector2();
        kLastSpeed = 0;
    }

    public Vector2 getTarget() {
        return kLastTarget;
    }

    public Vector2 getVelocity() {
        return kLastVelocity;
    }

    public Vector2 getDirection() {
        return kLastDirection;
    }

    public double getDistance() {
        return kLastDistance;
    }

    public double getRotation() {
        return kLastRotation;
    }

    public ShooterOdometryModel getModel() {
        return model;
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
        builder.addDoubleProperty("Rotation", this::getRotation, null);
    }

    protected double safe(double x) {
        if (!Double.isFinite(x))
            return 0;
        return x;
    }
}
