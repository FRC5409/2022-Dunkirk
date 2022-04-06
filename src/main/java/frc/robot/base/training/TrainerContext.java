package frc.robot.base.training;

import java.util.HashMap;
import java.util.Map;

import org.jetbrains.annotations.Nullable;

import frc.robot.base.Model4;
import frc.robot.base.Property;
import frc.robot.base.shooter.ShooterConfiguration;
import frc.robot.base.shooter.ShooterMode;
import frc.robot.base.shooter.odometry.ShooterOdometryModel;

public class TrainerContext {
    private Map<ShooterMode, TrainingConfiguration> configurations;
    private ShooterMode mode;

    public TrainerContext() {
        configurations = new HashMap<>();
        mode = null;
    }

    public void setConfiguration(ShooterMode mode, TrainingConfiguration configuration) {
        configurations.put(mode, configuration);
    }

    public void setMode(ShooterMode mode) {
        this.mode = mode;
    }

    @Nullable
    public TrainingConfiguration getConfiguration() {
        return configurations.get(mode);
    }
    
    @Nullable
    public TrainingConfiguration getConfiguration(ShooterMode mode) {
        return configurations.get(mode);
    }

    public ShooterMode getMode() {
        return mode;
    }

    public Property<ShooterConfiguration> getConfigurationProperty() {
        return new Property<ShooterConfiguration>() {
            @Override
            public ShooterConfiguration get() {
                return getConfiguration();
            }

            @Override
            public ShooterConfiguration set(ShooterConfiguration value) {
                return getConfiguration();
            }
        };
    }
}
