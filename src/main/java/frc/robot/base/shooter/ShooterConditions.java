package frc.robot.base.shooter;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class ShooterConditions implements Sendable {
    private int condition;

    public ShooterConditions() {
        this.condition = ShooterConditionType.kNone.value;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("kFlywheelReached", () -> hasCondition(ShooterConditionType.kFlywheelReached), null);
        builder.addBooleanProperty("kTurretReached", () -> hasCondition(ShooterConditionType.kTurretReached), null);
        builder.addBooleanProperty("kIndexerArmed", () -> hasCondition(ShooterConditionType.kIndexerArmed), null);
    }

    public void reset() {
        condition = ShooterConditionType.kNone.value;
    }

    public void addCondition(ShooterConditionType type) {
        this.condition |= type.value;
    }

    public void removeCondition(ShooterConditionType type) {
        this.condition &= ~type.value;
    }

    public boolean hasCondition(ShooterConditionType type) {
        return (condition & type.value) == type.value;
    }
}
