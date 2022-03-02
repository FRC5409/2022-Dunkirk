package frc.robot.base.shooter;

public class ShooterConfiguration {
    private final ShooterMode mode;
    private final ShooterModel model;
    private final VisionPipeline pipeline;

    public ShooterConfiguration(ShooterMode mode, VisionPipeline pipeline, ShooterModel model) {
        this.mode = mode;
        this.model = model;
        this.pipeline = pipeline;
    }

    public ShooterMode getMode() {
        return mode;
    }

    public ShooterModel getModel() {
        return model;
    }

    public VisionPipeline getPipeline() {
        return pipeline;
    }
}
