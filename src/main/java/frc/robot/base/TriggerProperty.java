package frc.robot.base;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TriggerProperty extends Trigger {
    private final CommandProperty<Boolean> property;

    public TriggerProperty() {
        this(false);
    }
    
    public TriggerProperty(boolean initialValue) {
        this(new CommandProperty<>(initialValue));
    }
    
    private TriggerProperty(CommandProperty<Boolean> property) {
        super(property::get);
        this.property = property;
    }

    public boolean set(boolean value) {
        return property.set(value);
    }

    public CommandProperty<Boolean> getProperty() {
        return property;
    }
}
