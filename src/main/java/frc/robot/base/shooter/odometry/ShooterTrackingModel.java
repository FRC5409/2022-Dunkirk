package frc.robot.base.shooter.odometry;

import frc.robot.base.shooter.target.FilterFactory;
import frc.robot.utils.Equation;
import frc.robot.utils.Gains;

public class ShooterTrackingModel {
    public final FilterFactory<?> kTargetFilter; 
    public final Equation kFlywheelModel;
    public final Equation kTurretModel;
    public final double kVelocityCompensation;
    public final double kRotationCompensation;
    public final double kAcquistionTimeout;
    public final double kDriveSpeed;
    public final double kTolerance;
    public final Gains kRotationGains;

    public ShooterTrackingModel(
        FilterFactory<?> kTargetFilter,
        double kAcquistionTimeout,
        double kVelocityCompensation,
        double kRotationCompensation,
        double kDriveSpeed,
        double kTolerance,
        Gains kRotationGains,
        Equation kFlywheelModel,
        Equation kTurretModel
    ) {
        this.kRotationCompensation = kRotationCompensation;
        this.kVelocityCompensation = kVelocityCompensation;
        this.kAcquistionTimeout = kAcquistionTimeout;
        this.kFlywheelModel = kFlywheelModel;
        this.kRotationGains = kRotationGains;
        this.kTargetFilter = kTargetFilter;
        this.kTurretModel = kTurretModel;
        this.kDriveSpeed = kDriveSpeed;
        this.kTolerance = kTolerance;
    }
}
