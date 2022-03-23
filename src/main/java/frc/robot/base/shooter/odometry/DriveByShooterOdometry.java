package frc.robot.base.shooter.odometry;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.utils.Equation;
import frc.robot.utils.Range;
import frc.robot.utils.Vector2;

/**
 * Experimental drive by shooter odometry
 */
public class DriveByShooterOdometry extends ActiveShooterOdometry {
    private final Equation kFlywheelOffsetMapping;
    private final Equation kTurretOffsetMapping;

    private double kLastTurretOffset;
    private double kLastFlywheelOffset;

    public DriveByShooterOdometry(
        ShooterOdometryModel model,
        Equation kFlywheelOffsetMapping,
        Equation kTurretOffsetMapping
    ) {
        super(model);
        
        kLastFlywheelOffset = 0;
        kLastTurretOffset = 0;

        this.kFlywheelOffsetMapping = kFlywheelOffsetMapping;
        this.kTurretOffsetMapping = kTurretOffsetMapping;
    }


    @Override
    public void update(Vector2 target, double speed, double rotation) {
        super.update(target, speed, rotation);

        kLastFlywheelOffset = kFlywheelOffsetMapping.calculate(
            Range.clamp(0, kLastDirection.x, 1));

        kLastTurretOffset = kTurretOffsetMapping.calculate(
            Range.clamp(0, kLastDirection.y, 1));
    }

    public void reset() {
        super.reset();

        kLastFlywheelOffset = 0;
        kLastTurretOffset = 0;
    }

    public double getFlywheelOffset() {
        return kLastFlywheelOffset;
    }
    
    public double getTurretOffset() {
        return kLastTurretOffset;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Turret Offset", this::getTurretOffset, null);
        builder.addDoubleProperty("Flywheel Offset", this::getFlywheelOffset, null);
    }
}
