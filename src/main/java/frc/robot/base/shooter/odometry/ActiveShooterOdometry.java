package frc.robot.base.shooter.odometry;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.base.shooter.target.FilterBase;
import frc.robot.utils.Vector2;
import frc.robot.utils.Vector3;

/**
 * Experimental shooter position relative odometry.
 */
public class ActiveShooterOdometry extends SimpleShooterOdometry {
    protected double  kLastSpeed;
    protected double  kLastRotation;
    protected Vector2 kLastVelocity;
    protected Vector2 kLastDirection;
    protected Vector2 kLastVisionDirection;

    public ActiveShooterOdometry(ShooterOdometryModel model, FilterBase filter) {
        super(model, filter);
        
        kLastVisionDirection = new Vector2();
        kLastDirection = new Vector2();
        kLastVelocity = new Vector2();
        kLastRotation = 0;
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
        Vector3 observerVector = calculateTargetProjection(filter.update(target));
        kLastDistance = safe(model.kHeight / Math.tan(Math.asin(observerVector.z)));

        Vector3 tempVector = observerVector.scale(kLastDistance)
           .sub(model.kViewOffset).unit();

        rotation = Math.toRadians(rotation);

        double cx = Math.cos(rotation);
        double cy = Math.sin(rotation);

        kLastVisionDirection = new Vector2(tempVector.x, tempVector.y).unit();
        kLastDirection = new Vector2(
            tempVector.x * cx - tempVector.y * cy,
            tempVector.x * cy + tempVector.y * cx
        ).unit();

        kLastRotation = safe(Math.acos(kLastDirection.x)) * Math.signum(kLastDirection.y);
        kLastVelocity = kLastDirection.scale(speed);
        kLastTarget = new Vector2(target);
        kLastUpdate = Timer.getFPGATimestamp();
        kLastSpeed = speed;
    }

    @Override
    public void reset() {
        super.reset();
        kLastVisionDirection = new Vector2();
        kLastDirection = new Vector2();
        kLastVelocity = new Vector2();
        kLastRotation = 0;
        kLastSpeed = 0;
    }

    public Vector2 getVisionDirection() {
        return kLastVisionDirection;
    }

    public Vector2 getVelocity() {
        return kLastVelocity;
    }

    public Vector2 getDirection() {
        return kLastDirection;
    }

    public double getRotation() {
        return Math.toDegrees(kLastRotation);
    }

    public double getSpeed() {
        return kLastSpeed;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addStringProperty("Velocity", () -> getVelocity().toString(), null);
        builder.addStringProperty("Direction", () -> getDirection().toString(), null);
        builder.addDoubleProperty("Speed", this::getSpeed, null);
        builder.addDoubleProperty("Rotation", () -> Math.toDegrees(this.getRotation()), null);
    }

    protected double safe(double x) {
        if (!Double.isFinite(x))
            return 0;
        return x;
    }
}
