package frc.robot.base;

import java.util.function.Consumer;
import java.util.function.Supplier;

import frc.robot.base.command.WaitConditionCommand;
import frc.robot.commands.ConfigureProperty;

public class CommandProperty<T> extends ValueProperty<T> {
    public CommandProperty(T value) {
        super(value);
    }

    public CommandProperty(Property<T> other) {
        super(other);
    }

    public ConfigureProperty<T> configureTo(T targetValue) {
        return new ConfigureProperty<>(this, targetValue);
    }
    
    public ConfigureProperty<T> configureTo(Supplier<T> supplier) {
        return new ConfigureProperty<>(this, supplier);
    }
    
    public ConfigureProperty<T> configureTo(Consumer<Property<T>> consumer) {
        return new ConfigureProperty<>(this, consumer);
    }

    public WaitConditionCommand equalTo(T expectedValue) {
        return new WaitConditionCommand(() -> m_value.equals(expectedValue));
    }

    public WaitConditionCommand equalTo(Supplier<T> expectedValue) {
        return new WaitConditionCommand(() -> m_value.equals(expectedValue.get()));
    }

    public WaitConditionCommand notEqualTo(T expectedValue) {
        return new WaitConditionCommand(() -> !m_value.equals(expectedValue));
    }

    public WaitConditionCommand notEqualTo(Supplier<T> expectedValue) {
        return new WaitConditionCommand(() -> !m_value.equals(expectedValue.get()));
    }
}
