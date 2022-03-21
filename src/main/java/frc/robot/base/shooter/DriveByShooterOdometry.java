package frc.robot.base.shooter;

import frc.robot.utils.Equation;
import frc.robot.utils.Vector2;

/**
 * Experimental drive by shooter odometry
 */
public class DriveByShooterOdometry {
    private final ShooterOdometry odometry;
    private final Equation kFlywheelOffsetMapping;
    private final Equation kTurretOffsetMapping;

    private double kLastTurretOffset;
    private double kLastFlywheelOffset;

    public DriveByShooterOdometry(
        ShooterOdometry odometry,
        Equation kFlywheelOffsetMapping,
        Equation kTurretOffsetMapping
    ) {
        this.odometry = odometry;
        this.kFlywheelOffsetMapping = kFlywheelOffsetMapping;
        this.kTurretOffsetMapping = kTurretOffsetMapping;
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

    public double getFlywheelOffset() {
        return kLastFlywheelOffset;
    }
    
    public double getTurretOffset() {
        return kLastTurretOffset;
    }
}
