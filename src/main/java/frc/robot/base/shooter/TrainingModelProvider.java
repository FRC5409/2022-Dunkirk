package frc.robot.base.shooter;

import frc.robot.base.shooter.odometry.ShooterOdometryModel;
import frc.robot.base.shooter.odometry.ShooterTrackingModel;
import frc.robot.base.training.TrainingModel4;

public class TrainingModelProvider implements ShooterModelProvider {
    private final ShooterOdometryModel odometryModel;
    private final ShooterTrackingModel trackingModel;
    private final TrainingModel4 trainingModel;

    public TrainingModelProvider(
        ShooterOdometryModel odometryModel,
        ShooterTrackingModel trackingModel,
        TrainingModel4 trainingModel
    ) {
        this.odometryModel = odometryModel;
        this.trackingModel = trackingModel;
        this.trainingModel = trainingModel;
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
    public TrainingModel4 getExecutionModel() {
        return trainingModel;
    }
}
