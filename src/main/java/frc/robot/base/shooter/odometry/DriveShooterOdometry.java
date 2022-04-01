package frc.robot.base.shooter.odometry;

import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.utils.Vector2;

/**
 * Experimental drive by shooter odometry
 */
public class DriveShooterOdometry extends ActiveShooterOdometry {
    private double kLastTurretOffset;
    private double kLastFlywheelOffset;

    public DriveShooterOdometry(
        ShooterOdometryModel odometryModel,
        ShooterTrackingModel trackingModel
    ) {
        super(odometryModel, trackingModel);
        
        kLastFlywheelOffset = 0;
        kLastTurretOffset = 0;
    }


    @Override
    public void update(Vector2 target, double speed, double rotation) {
        super.update(target, speed, rotation);

        kLastFlywheelOffset = trackingModel.kFlywheelModel.calculate(kLastVelocity.x);
        kLastTurretOffset = trackingModel.kTurretModel.calculate(kLastVelocity.y);
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
