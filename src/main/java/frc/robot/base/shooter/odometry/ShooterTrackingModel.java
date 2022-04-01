package frc.robot.base.shooter.odometry;

import frc.robot.base.shooter.target.FilterFactory;
import frc.robot.utils.Equation;

public class ShooterTrackingModel {
    public final FilterFactory<?> kTargetFilter; 
    public final Equation kFlywheelModel;
    public final Equation kTurretModel;
    public final double kAcquistionTimeout;
    public final double kMinOutput;
    public final double kDriveSpeed;

    public ShooterTrackingModel(
        FilterFactory<?> kTargetFilter,
        double kAcquistionTimeout,
        double kMinOutput,
        double kDriveSpeed,
        Equation kFlywheelModel,
        Equation kTurretModel
    ) {
        this.kAcquistionTimeout = kAcquistionTimeout;
        this.kFlywheelModel = kFlywheelModel;
        this.kTargetFilter = kTargetFilter;
        this.kTurretModel = kTurretModel;
        this.kDriveSpeed = kDriveSpeed;
        this.kMinOutput = kMinOutput;
    }
}
