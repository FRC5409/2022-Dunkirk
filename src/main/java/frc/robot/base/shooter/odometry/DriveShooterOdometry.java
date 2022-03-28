package frc.robot.base.shooter.odometry;

import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.base.shooter.target.FilterBase;
import frc.robot.utils.Equation;
import frc.robot.utils.Range;
import frc.robot.utils.Vector2;

/**
 * Experimental drive by shooter odometry
 */
public class DriveShooterOdometry extends ActiveShooterOdometry {
    private final Equation flywheelOffsetModel;
    private final Equation turretOffsetModel;

    private double kLastTurretOffset;
    private double kLastFlywheelOffset;

    public DriveShooterOdometry(
        ShooterOdometryModel model,
        FilterBase filter,
        Equation flywheelOffsetModel,
        Equation turretOffsetModel
    ) {
        super(model, filter);
        
        kLastFlywheelOffset = 0;
        kLastTurretOffset = 0;

        this.flywheelOffsetModel = flywheelOffsetModel;
        this.turretOffsetModel = turretOffsetModel;
    }


    @Override
    public void update(Vector2 target, double speed, double rotation) {
        super.update(target, speed, rotation);

        kLastFlywheelOffset = flywheelOffsetModel.calculate(kLastVelocity.x);
        kLastTurretOffset = turretOffsetModel.calculate(kLastVelocity.y);
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