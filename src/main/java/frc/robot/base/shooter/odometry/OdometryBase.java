package frc.robot.base.shooter.odometry;

import edu.wpi.first.util.sendable.Sendable;
import frc.robot.utils.Matrix3;
import frc.robot.utils.Vector2;
import frc.robot.utils.Vector3;

public abstract class OdometryBase implements Sendable {
    protected final ShooterOdometryModel model;
    
    protected final Matrix3 kViewProjection;

    public OdometryBase(ShooterOdometryModel model) {
        double kPitch = -Math.toRadians(model.kPitch);
        
        kViewProjection = new Matrix3(
             Math.cos(kPitch), 0d, Math.sin(kPitch),
             0d,               1d,               0d,
            -Math.sin(kPitch), 0d, Math.cos(kPitch)
        );
        
        this.model = model;
    }

    public abstract void reset();

    protected Vector3 calculateTargetProjection(Vector2 target) {
        Vector2 viewTarget = new Vector2(
            Math.toRadians(target.x),
            Math.toRadians(target.y)
        );
        
        double cy = Math.cos(viewTarget.y);

        return kViewProjection.apply(
            new Vector3(
                Math.cos(viewTarget.x) * cy,
                Math.sin(viewTarget.x) * cy,
                Math.sin(viewTarget.y)
            )
        ).unit();
    }
}
