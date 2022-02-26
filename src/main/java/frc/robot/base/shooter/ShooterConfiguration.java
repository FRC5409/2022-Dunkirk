package frc.robot.base.shooter;

public class ShooterConfiguration {
    private ShooterMode mode;
    private ShooterModel model;

    public ShooterConfiguration(ShooterMode mode, ShooterModel model) {
        this.mode = mode;
        this.model = model;
    }

    public ShooterMode getMode() {
        return mode;
    }

    public ShooterModel getModel() {
        return model;
    }
}
