package frc.robot.base.shooter;

import frc.robot.base.shooter.odometry.ShooterOdometryModel;
import frc.robot.base.shooter.odometry.ShooterTrackingModel;

public class TrainingModelProvider implements ShooterModelProvider {
    private final ShooterOdometryModel odometryModel;
    private final ShooterTrackingModel trackingModel;
    private final ShooterTrainingModel4 trainingModel;

    public TrainingModelProvider(
        ShooterOdometryModel odometryModel,
        ShooterTrackingModel trackingModel,
        ShooterTrainingModel4 trainingModel
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
    public ShooterTrainingModel4 getExecutionModel() {
        return trainingModel;
    }
}
