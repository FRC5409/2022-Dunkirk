package frc.robot.base;

import org.jetbrains.annotations.Nullable;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;

public abstract class StateCommandBase extends CommandBase implements StateCommand {
    @Nullable
    protected String m_next;

    @Override
    public void next(String name) {
        m_next = name;
    }

    @Override
    public void reset() {
        m_next = null;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addStringProperty("State Name", this::getStateName, null);
        builder.addStringProperty("Next State", this::getNextState, null);
    }

    @Override
    @Nullable
    public String getNextState() {
        return m_next;
    }
}
