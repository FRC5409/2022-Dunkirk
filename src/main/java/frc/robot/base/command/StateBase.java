package frc.robot.base.command;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.Subsystem;

public abstract class StateBase implements State {
    protected Set<Subsystem> m_requirements = new HashSet<>();
    protected String m_next = null;
    protected int    m_executionIndex = -1;


    protected StateBase() {
        StateCommandManager.getInstance().addState(this);
    }
        
    public final void addRequirements(Subsystem... requirements) {
        m_requirements.addAll(Set.of(requirements));
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
    public void setExecutionIndex(int index) {
        m_executionIndex = index;
    }

    @Override
    public String getNextState() {
        return m_next;
    }

    @Override
    public int getExecutionIndex() {
        return m_executionIndex;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return m_requirements;
    }

    /**
     * If the state is not operating at the top of the execution
     * stack.
     * @return If the state is dormant
     */
    public final boolean isDormant() {
        if (m_executionIndex == -1)
            throw new RuntimeException("Cannot check dormancy when state is not executing");
        return m_executionIndex != 0;
    }
}
