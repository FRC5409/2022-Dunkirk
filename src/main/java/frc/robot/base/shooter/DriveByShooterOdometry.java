package frc.robot.base.shooter;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.utils.Equation;
import frc.robot.utils.Vector2;

/**
 * Experimental drive by shooter odometry
 */
public class DriveByShooterOdometry implements Sendable {
    private final ActiveShooterOdometry odometry;
    private final Equation kFlywheelOffsetMapping;
    private final Equation kTurretOffsetMapping;

    private double kLastTurretOffset;
    private double kLastFlywheelOffset;

    public DriveByShooterOdometry(
        ShooterOdometryModel model,
        Equation kFlywheelOffsetMapping,
        Equation kTurretOffsetMapping
    ) {
        this.odometry = new ActiveShooterOdometry(model);
        this.kFlywheelOffsetMapping = kFlywheelOffsetMapping;
        this.kTurretOffsetMapping = kTurretOffsetMapping;

        reset();
    }

    /**
     * 
     * @param target   The observed vision target
     * @param speed    The observed speed
     * @param rotation The observed view rotation
     */
    public void update(Vector2 target, double speed, double rotation) {
        odometry.update(target, speed, rotation);

        Vector2 unitVelocity = odometry.getVelocity().unit();

        kLastFlywheelOffset = kFlywheelOffsetMapping.calculate(unitVelocity.x);
        kLastTurretOffset = kTurretOffsetMapping.calculate(unitVelocity.y);
    }

    public void reset() {
        odometry.reset();

        kLastFlywheelOffset = 0;
        kLastTurretOffset = 0;
    }

    public Vector2 getVelocity() {
        return odometry.getVelocity();
    }

    public double getSpeed() {
        return odometry.getSpeed();
    }

    public double getDistance() {
        return odometry.getDistance();
    }

    public Vector2 getTarget() {
        return odometry.getTarget();
    }

    public double getRotation() {
        return odometry.getRotation();
    }

    public double getFlywheelOffset() {
        return kLastFlywheelOffset;
    }
    
    public double getTurretOffset() {
        return kLastTurretOffset;
    }

    public ShooterOdometryModel getModel() {
        return odometry.getModel();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Velocity", () -> getVelocity().toString(), null);
        builder.addStringProperty("Target", () -> getTarget().toString(), null);
        builder.addDoubleProperty("Distance", this::getDistance, null);
        builder.addDoubleProperty("Speed", this::getSpeed, null);
        builder.addDoubleProperty("Rotation", this::getRotation, null);
        builder.addDoubleProperty("Turret Offset", this::getTurretOffset, null);
        builder.addDoubleProperty("Flywheel Offset", this::getFlywheelOffset, null);
    }
}
