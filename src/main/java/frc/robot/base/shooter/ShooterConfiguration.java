package frc.robot.base.shooter;

import org.jetbrains.annotations.Nullable;

public class ShooterConfiguration {
    private final ShooterExecutionModel executionModel;
    private final ShooterOdometryModel odometryModel;
    private final VisionPipeline visionPipeline;
    private final HoodPosition hoodConfiguration;
    private final ShooterMode shooterMode;

    public ShooterConfiguration(
        ShooterMode shooterMode,
        HoodPosition hoodConfiguration,
        VisionPipeline visionPipeline,
        ShooterOdometryModel odometryModel
    ) {
        this.hoodConfiguration = hoodConfiguration;
        this.visionPipeline = visionPipeline;
        this.executionModel = null;
        this.odometryModel = odometryModel;
        this.shooterMode = shooterMode;
    }

    public ShooterConfiguration(
        ShooterMode shooterMode,
        HoodPosition hoodConfiguration,
        VisionPipeline visionPipeline,
        ShooterOdometryModel odometryModel,
        ShooterExecutionModel executionModel
    ) {
        this.hoodConfiguration = hoodConfiguration;
        this.visionPipeline = visionPipeline;
        this.executionModel = executionModel;
        this.odometryModel = odometryModel;
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
    public ShooterExecutionModel getExecutionModel() {
        return executionModel;
    }

    public ShooterOdometryModel getOdometryModel() {
        return odometryModel;
    }
}
