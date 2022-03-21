package frc.robot.base.shooter;

import frc.robot.utils.Matrix3;
import frc.robot.utils.Vector2;
import frc.robot.utils.Vector3;

/**
 * Experimental shooter position relative odometry.
 */
public class SimpleShooterOdometry {
    private final ShooterOdometryModel model;
    
    private final Matrix3 kViewProjection;
    private final double  kViewConversionFactor;

    private double kLastDistance;
    private Vector2 kLastTarget;

    public SimpleShooterOdometry(ShooterOdometryModel model) {
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
     */
    public void update(Vector2 target) {
        final Vector3 observerVector = kViewProjection.apply(
            new Vector3(
                Math.cos(target.x * kViewConversionFactor),
                Math.sin(target.x * kViewConversionFactor),
                0d
            )
        ).unit();

        kLastDistance = model.kHeight / Math.tan(Math.asin(observerVector.z));
        if (!Double.isFinite(kLastDistance))
            kLastDistance = 0;

        kLastTarget = new Vector2(target);
    }

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
}
