package frc.robot.commands;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.base.Property;

public class ConfigureProperty<T> extends CommandBase {
    private final Consumer<Property<T>> consumer;
    private final Property<T> property;
    
    public ConfigureProperty(Property<T> property, Consumer<Property<T>> consumer) {
        this.property = property;
        this.consumer = consumer;
    }

    public ConfigureProperty(Property<T> property, Supplier<T> supplier) {
        this(property, p -> p.set(supplier.get()));
    }

    public ConfigureProperty(Property<T> property, T value) {
        this(property, p -> p.set(value));
    }

    @Override
    public void initialize() {
        consumer.accept(property);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
    public Property<T> getProperty() {
        return property;
    }
}
