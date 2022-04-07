package frc.robot.base.shooter.odometry;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.base.shooter.TrackingGains;
import frc.robot.base.shooter.target.FilterFactory;
import frc.robot.utils.Equation;

public class ShooterTrackingModel implements Sendable {
    public final FilterFactory<?> kTargetFilter; 
    public final TrackingGains kGains;
    public final Equation kFlywheelModel;
    public final Equation kTurretModel;
    public double kAcquistionTimeout;
    public double kDriveSpeed;
    public double kTolerance;

    public ShooterTrackingModel(
        FilterFactory<?> kTargetFilter,
        TrackingGains kGains,
        double kAcquistionTimeout,
        double kDriveSpeed,
        double kTolerance,
        Equation kFlywheelModel,
        Equation kTurretModel
    ) {
        this.kAcquistionTimeout = kAcquistionTimeout;
        this.kFlywheelModel = kFlywheelModel;
        this.kTargetFilter = kTargetFilter;
        this.kTurretModel = kTurretModel;
        this.kDriveSpeed = kDriveSpeed;
        this.kTolerance = kTolerance;
        this.kGains = kGains;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("kAcquistionTimeout", () -> kAcquistionTimeout, x -> kAcquistionTimeout = x);
        builder.addDoubleProperty("kDriveSpeed", () -> kDriveSpeed, x -> kDriveSpeed = x);
        builder.addDoubleProperty("kTolerance", () -> kTolerance, x -> kTolerance = x);
    }
}
