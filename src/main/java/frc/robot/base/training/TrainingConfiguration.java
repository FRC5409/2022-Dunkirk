package frc.robot.base.training;

import org.jetbrains.annotations.Nullable;

import frc.robot.Constants;
import frc.robot.base.shooter.HoodPosition;
import frc.robot.base.shooter.ShooterConfiguration;
import frc.robot.base.shooter.ShooterMode;
import frc.robot.base.shooter.VisionPipeline;
import frc.robot.base.shooter.odometry.ShooterOdometryModel;
import frc.robot.base.shooter.odometry.ShooterTrackingModel;

public class TrainingConfiguration implements ShooterConfiguration {
    private final ShooterTrackingModel trackingModel;
    private final ShooterOdometryModel odometryModel;
    private final VisionPipeline visionPipeline;
    private final HoodPosition hoodConfiguration;
    private final ShooterMode shooterMode;
    
    private TrainingModel4 executionModel;
    private double distance;
    
    public TrainingConfiguration(
        ShooterMode shooterMode,
        HoodPosition hoodConfiguration,
        VisionPipeline visionPipeline,
        ShooterTrackingModel trackingModel,
        ShooterOdometryModel odometryModel
    ) {
        this.hoodConfiguration = hoodConfiguration;
        this.visionPipeline = visionPipeline;
        this.executionModel = new TrainingModel4(0.0, 0.0, 0.0, 0.0, Constants.Shooter.DISTANCE_RANGE, Constants.Shooter.SPEED_RANGE);
        this.odometryModel = odometryModel;
        this.trackingModel = trackingModel;
        this.shooterMode = shooterMode;
        this.distance = 0;
    }
    
    public void setDistance(double distance) {
        this.distance = distance;
    }

    public double getDistance() {
        return distance;
    }

    @Override
    public ShooterMode getMode() {
        return shooterMode;
    }

    @Override
    public VisionPipeline getPipeline() {
        return visionPipeline;
    }

    @Override
    public HoodPosition getHoodPosition() {
        return hoodConfiguration;
    }

    @Override
    @Nullable
    public TrainingModel4 getExecutionModel() {
        return executionModel;
    }

    @Override
    public ShooterOdometryModel getOdometryModel() {
        return odometryModel;
    }

    @Override
    public ShooterTrackingModel getTrackingModel() {
        return trackingModel;
    }
}
