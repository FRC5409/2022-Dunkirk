package frc.robot.base.shooter;

import org.jetbrains.annotations.Nullable;

import frc.robot.base.Model4;
import frc.robot.base.shooter.odometry.ShooterOdometryModel;
import frc.robot.base.shooter.odometry.ShooterTrackingModel;
import frc.robot.base.shooter.target.FilterFactory;

public class ShooterConfiguration {
    private final Model4 executionModel;
    private final ShooterTrackingModel trackingModel;
    private final ShooterOdometryModel odometryModel;
    private final VisionPipeline visionPipeline;
    private final HoodPosition hoodConfiguration;
    private final ShooterMode shooterMode;
    
    public ShooterConfiguration(
        ShooterMode shooterMode,
        HoodPosition hoodConfiguration,
        VisionPipeline visionPipeline,
        ShooterTrackingModel trackingModel,
        ShooterOdometryModel odometryModel
    ) {
        this.hoodConfiguration = hoodConfiguration;
        this.visionPipeline = visionPipeline;
        this.executionModel = null;
        this.trackingModel = trackingModel;
        this.odometryModel = odometryModel;
        this.shooterMode = shooterMode;
    }

    public ShooterConfiguration(
        ShooterMode shooterMode,
        HoodPosition hoodConfiguration,
        VisionPipeline visionPipeline,
        FilterFactory<?> targetFilterFactory,
        ShooterTrackingModel trackingModel,
        ShooterOdometryModel odometryModel,
        Model4 executionModel
    ) {
        this.hoodConfiguration = hoodConfiguration;
        this.visionPipeline = visionPipeline;
        this.executionModel = executionModel;
        this.odometryModel = odometryModel;
        this.trackingModel = trackingModel;
        this.shooterMode = shooterMode;
    }

    public ShooterMode getMode() {
        return shooterMode;
    }

    public VisionPipeline getPipeline() {
        return visionPipeline;
    }

    public HoodPosition getHoodPosition() {
        return hoodConfiguration;
    }

    @Nullable
    public Model4 getExecutionModel() {
        return executionModel;
    }

    public ShooterOdometryModel getOdometryModel() {
        return odometryModel;
    }

    public ShooterTrackingModel getTrackingModel() {
        return trackingModel;
    }
}
