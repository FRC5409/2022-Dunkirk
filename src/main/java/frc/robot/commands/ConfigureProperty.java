package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.base.Property;

public class ConfigureProperty<T> extends CommandBase {
    private final Property<T> property;
    private final T           value;

    public ConfigureProperty(Property<T> property, T value) {
        this.property = property;
        this.value    = value;
    }

    @Override
    public void initialize() {
        property.set(value);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
