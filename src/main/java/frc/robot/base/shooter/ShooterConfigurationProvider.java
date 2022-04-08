package frc.robot.base.shooter;

import java.util.HashMap;
import java.util.Map;
import java.util.Set;

public class ShooterConfigurationProvider {
    private final Map<ShooterMode, ShooterConfiguration> configurations;

    public static ShooterConfigurationProvider of(ShooterConfiguration... configs) {
        return new ShooterConfigurationProvider(Set.of(configs));
    }
    
    public ShooterConfigurationProvider(Set<ShooterConfiguration> configs) {
        configurations = new HashMap<>();
        for (ShooterConfiguration config : configs) {
            configurations.put(config.getMode(), config);
        }
    }

    public ShooterConfiguration getConfiguration(ShooterMode mode) {
        return configurations.get(mode);
    }
}
