package frc.robot.base.shooter;

import frc.robot.utils.Matrix3;
import frc.robot.utils.Vector2;
import frc.robot.utils.Vector3;

/**
 * Experimental shooter position relative odometry.
 */
public class ShooterOdometry {
    private final ShooterOdometryModel model;
    
    private final Matrix3 kViewProjection;
    private final double  kViewConversionFactor;
    
    private double  kLastSpeed;
    private Vector2 kLastTarget;
    private Vector2 kLastVelocity;
    private double  kLastDistance;

    public ShooterOdometry(ShooterOdometryModel model) {
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
        kLastVelocity = new Vector2();
        kLastTarget = new Vector2();

        kLastDistance = 0;
        kLastSpeed = 0;
    }

    public Vector2 getVelocity() {
        return kLastVelocity;
    }

    public double getDistance() {
        return kLastDistance;
    }
}
