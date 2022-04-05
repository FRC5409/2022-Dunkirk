package frc.robot.base.shooter;

import org.jetbrains.annotations.Nullable;

import frc.robot.base.shooter.odometry.ShooterOdometryModel;
import frc.robot.base.shooter.odometry.ShooterTrackingModel;
import frc.robot.utils.Equation;

public interface ShooterConfiguration {
    ShooterMode getMode();

    VisionPipeline getPipeline();

    HoodPosition getHoodPosition();

    @Nullable
    Equation getExecutionModel();

    ShooterOdometryModel getOdometryModel();

    ShooterTrackingModel getTrackingModel();
}
