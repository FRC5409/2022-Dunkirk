package frc.robot.base.shooter;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.utils.Matrix3;
import frc.robot.utils.Vector2;
import frc.robot.utils.Vector3;

/**
 * Experimental shooter position relative odometry.
 */
public class ActiveShooterOdometry implements Sendable {
    private final ShooterOdometryModel model;
    
    private final Matrix3 kViewProjection;
    private final double  kViewConversionFactor;
    
    private double  kLastSpeed;
    private Vector2 kLastTarget;
    private Vector2 kLastVelocity;
    private double  kLastDistance;
    private double  kLastRotation;

    public ActiveShooterOdometry(ShooterOdometryModel model) {
        double kPitch = -Math.toRadians(model.kPitch);
        
        kViewConversionFactor = Math.toRadians(model.kFieldOfView.x / 2d);
        kViewProjection = new Matrix3(
             Math.cos(kPitch), 0d, Math.sin(kPitch),
             0d,               1d,               0d,
            -Math.sin(kPitch), 0d, Math.cos(kPitch)
        );
        
        reset();
        
        this.model = model;
    }

    /**
     * 
     * @param target   The observed vision target
     * @param speed    The observed speed
     * @param rotation The observed view rotation
     */
    public void update(Vector2 target, double speed, double rotation) {
        final Vector3 observerVector = kViewProjection.apply(
            new Vector3(
                Math.cos(target.x * kViewConversionFactor),
                Math.sin(target.x * kViewConversionFactor),
                0d
            )
        ).unit();

        rotation = Math.toRadians(rotation);
        kLastVelocity = new Vector2(
            observerVector.x + Math.cos(rotation),
            observerVector.y + Math.sin(rotation)
        ).unit().scale(speed);

        kLastDistance = model.kHeight / Math.tan(Math.asin(observerVector.z));
        if (!Double.isFinite(kLastDistance))
            kLastDistance = 0;

        kLastTarget = new Vector2(target);
        kLastSpeed = speed;
    }

    public void reset() {
        kLastDistance = 0;
        kLastRotation = 0;
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
        builder.addStringProperty("Target", () -> getTarget().toString(), null);
        builder.addDoubleProperty("Distance", this::getDistance, null);
        builder.addDoubleProperty("Speed", this::getSpeed, null);
        builder.addDoubleProperty("Rotation", this::getRotation, null);
    }
}
