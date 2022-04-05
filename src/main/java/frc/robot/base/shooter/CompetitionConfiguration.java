package frc.robot.base.shooter;

import org.jetbrains.annotations.Nullable;

import frc.robot.base.Model4;
import frc.robot.base.shooter.odometry.ShooterOdometryModel;
import frc.robot.base.shooter.odometry.ShooterTrackingModel;
import frc.robot.utils.Equation;

public class CompetitionConfiguration implements ShooterConfiguration {
    private final ShooterTrackingModel trackingModel;
    private final ShooterOdometryModel odometryModel;
    private final VisionPipeline visionPipeline;
    private final HoodPosition hoodConfiguration;
    private final ShooterMode shooterMode;
    private final Model4 executionModel;
    
    public CompetitionConfiguration(
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

    public CompetitionConfiguration(
        ShooterMode shooterMode,
        HoodPosition hoodConfiguration,
        VisionPipeline visionPipeline,
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
    public Equation getExecutionModel() {
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
