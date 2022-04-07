package frc.robot.base.shooter;

public class ShooterConfiguration {
    private final VisionPipeline visionPipeline;
    private final HoodPosition hoodConfiguration;
    private final ShooterMode shooterMode;
    
    public ShooterConfiguration(
        ShooterMode shooterMode,
        HoodPosition hoodConfiguration,
        VisionPipeline visionPipeline
    ) {
        this.hoodConfiguration = hoodConfiguration;
        this.visionPipeline = visionPipeline;
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
}
