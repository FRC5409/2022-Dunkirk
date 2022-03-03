package frc.robot.base.shooter;

import org.jetbrains.annotations.Nullable;

public class ShooterConfiguration {
    private final ShooterMode mode;
    private final HoodPosition hood;
    private final ShooterModel model;
    private final VisionPipeline pipeline;

    public ShooterConfiguration(
        ShooterMode mode,
        VisionPipeline pipeline,
        HoodPosition hood
    ) {
        this(mode, pipeline, hood, null);
    }

    public ShooterConfiguration(
        ShooterMode mode,
        VisionPipeline pipeline,
        HoodPosition hood,
        @Nullable ShooterModel model
    ) {
        this.mode = mode;
        this.hood = hood;
        this.model = model;
        this.pipeline = pipeline;
    }

    public ShooterMode getMode() {
        return mode;
    }

    public VisionPipeline getPipeline() {
        return pipeline;
    }

    public HoodPosition getHoodPosition() {
        return hood;
    }

    @Nullable
    public ShooterModel getModel() {
        return model;
    }
}
