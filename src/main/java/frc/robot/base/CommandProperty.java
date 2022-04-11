package frc.robot.base;

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

    public WaitConditionCommand equalTo(T expectedValue) {
        return new WaitConditionCommand(() -> m_value == expectedValue);
    }

    public WaitConditionCommand notEqualTo(T expectedValue) {
        return new WaitConditionCommand(() -> m_value != expectedValue);
    }
}
