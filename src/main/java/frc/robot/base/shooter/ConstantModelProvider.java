package frc.robot.base.shooter;

import frc.robot.base.shooter.odometry.ShooterOdometryModel;
import frc.robot.base.shooter.odometry.ShooterTrackingModel;
import frc.robot.utils.Equation;

public class ConstantModelProvider implements ShooterModelProvider {
    private final ShooterOdometryModel odometryModel;
    private final ShooterTrackingModel trackingModel;
    private final Equation executionModel;

    public ConstantModelProvider(
        ShooterOdometryModel odometryModel,
        ShooterTrackingModel trackingModel,
        Equation executionModel
    ) {
        this.odometryModel = odometryModel;
        this.trackingModel = trackingModel;
        this.executionModel = executionModel;
    }
    @Override
    public ShooterOdometryModel getOdometryModel() {
        return odometryModel;
    }

    @Override
    public ShooterTrackingModel getTrackingModel() {
        return trackingModel;
    }

    @Override
    public Equation getExecutionModel() {
        return executionModel;
    }
}
