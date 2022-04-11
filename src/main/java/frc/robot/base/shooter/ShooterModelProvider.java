package frc.robot.base.shooter;

import frc.robot.base.shooter.odometry.ShooterOdometryModel;
import frc.robot.base.shooter.odometry.ShooterTrackingModel;
import frc.robot.utils.Equation;

public interface ShooterModelProvider {
    public ShooterOdometryModel getOdometryModel();
    public ShooterTrackingModel getTrackingModel();
    public Equation getExecutionModel();
}
