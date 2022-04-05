package frc.robot.base.shooter.odometry;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
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
    protected Vector2 kLastViewDirection;

    public ActiveShooterOdometry(
        ShooterOdometryModel odometryModel,
        ShooterTrackingModel trackingModel
    ) {
        super(odometryModel, trackingModel);
        
        kLastViewDirection = new Vector2();
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
        Vector3 observerVector = calculateTargetProjection(kFilter.update(target));
        kLastDistance = safe(odometryModel.kHeight / Math.tan(Math.asin(observerVector.z))) + odometryModel.kOffset;

        Vector3 tempVector = observerVector.scale(kLastDistance)
           .sub(odometryModel.kViewOffset).unit();

        rotation = Math.toRadians(rotation);

        double cx = Math.cos(rotation);
        double cy = Math.sin(rotation);

        kLastViewDirection = new Vector2(tempVector.x, tempVector.y).unit();

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
        kLastViewDirection = new Vector2();
        kLastDirection = new Vector2();
        kLastVelocity = new Vector2();
        kLastRotation = 0;
        kLastSpeed = 0;
    }

    public Vector2 getViewDirection() {
        return kLastViewDirection;
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
