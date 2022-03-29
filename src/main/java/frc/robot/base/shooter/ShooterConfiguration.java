package frc.robot.base.shooter;

import org.jetbrains.annotations.Nullable;

import frc.robot.base.Model4;
import frc.robot.base.shooter.odometry.ShooterOdometryModel;
import frc.robot.base.shooter.target.FilterBase;
import frc.robot.base.shooter.target.FilterFactory;

public class ShooterConfiguration {
    private final Model4 executionModel;
    private final ShooterOdometryModel odometryModel;
    private final FilterFactory<?> targetFilterFactory;
    private final VisionPipeline visionPipeline;
    private final HoodPosition hoodConfiguration;
    private final ShooterMode shooterMode;
    
    public ShooterConfiguration(
        ShooterMode shooterMode,
        HoodPosition hoodConfiguration,
        VisionPipeline visionPipeline,
        FilterFactory<?> targetFilterFactory,
        ShooterOdometryModel odometryModel
    ) {
        this.targetFilterFactory = targetFilterFactory;
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
        FilterFactory<?> targetFilterFactory,
        ShooterOdometryModel odometryModel,
        Model4 executionModel
    ) {
        this.targetFilterFactory = targetFilterFactory;
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
    public Model4 getExecutionModel() {
        return executionModel;
    }

    public ShooterOdometryModel getOdometryModel() {
        return odometryModel;
    }

    public FilterBase getTargetFilter() {
        return targetFilterFactory.create();
    }
}
