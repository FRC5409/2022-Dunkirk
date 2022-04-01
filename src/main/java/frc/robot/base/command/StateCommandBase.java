package frc.robot.base.command;

import org.jetbrains.annotations.Nullable;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;

public abstract class StateCommandBase extends CommandBase implements StateCommand {
    @Nullable
    protected String m_next = null;
    protected int    m_executionIndex = -1;

    protected StateCommandBase() {
        StateCommandManager.getInstance().addState(this);
    }
    
    @Override
    public void next(String name) {
        m_next = name;
    }

    @Override
    public void reset() {
        m_next = null;
        m_executionIndex = -1;
    }

    @Override
    public final void schedule(boolean interruptible) throws UnsupportedOperationException {
        throw new UnsupportedOperationException("Cannot schedule a state command.");
    }

    @Override
    public void setExecutionIndex(int index) {
        m_executionIndex = index;
    }

    @Override
    @Nullable
    public String getNextState() {
        return m_next;
    }

    @Override
    public int getExecutionIndex() {
        return m_executionIndex;
    }

    /**
     * If the state is not operating at the top of the execution
     * stack.
     * @return If the state is dormant
     */
    public final boolean isDormant() {
        if (m_executionIndex == -1)
            throw new RuntimeException("Cannot checky dormancy when state while state is not executing");
        return m_executionIndex != 0;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addStringProperty("State Name", this::getStateName, null);
        builder.addStringProperty("Next State", this::getNextState, null);
    }
}
