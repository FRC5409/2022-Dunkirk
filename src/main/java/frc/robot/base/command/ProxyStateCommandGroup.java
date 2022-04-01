package frc.robot.base.command;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.jetbrains.annotations.Nullable;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * <h2> ProxyStateCommandGroupTest </h2>
 * A command-based state machine representing a series of tasks to be performed by the robot
 * in an independant execution order.
 * 
 * <p>When the {@link ProxyStateCommandGroup} executes, it schedules states without obtaining
 * their requirements, as opposed to the typical behaviour of a {@link StateCommandGroup}.</p>
 * 
 * <p>Suppose a state group with states [A, B, C, D]. If state 'A' requires a given subsystem,
 * the state will only obtain ownership of the subsytem upon its individual execution, as opposed
 * to the execution of the group.</p>
 * 
 * @author Keith Davies
 * @see StateCommand, StateCommandGroup
 */
public abstract class ProxyStateCommandGroup extends CommandBase {
    protected Map<String, StateCommand> m_states;
    protected StateExecutionStack m_stack;
    protected StateCommand m_default;

    /**
     * Construct an empty {@link ProxyStateCommandGroup}.
     */
    public ProxyStateCommandGroup() {
        m_default = null;
        m_states = new HashMap<>();
        m_stack = null;
    }
    
    /**
     * Construct a {@link ProxyStateCommandGroup} with several states.
     * 
     * @param commands The states included in the group.
     */
    public ProxyStateCommandGroup(StateCommand... commands) {
        this();
        addStates(commands);
    }

    
    /**
     * Construct a {@link ProxyStateCommandGroup} with several states,
     * as well as a default state.
     * 
     * @param commands         The states included in the group.
     * @param defaultStateName The default state name.
     */
    public ProxyStateCommandGroup(String defaultStateName, StateCommand... commands) {
        this();
        addStates(commands);
        m_default = getState(defaultStateName);
    }

    @Override
    public void initialize() {
        if (m_default == null) {
            System.err.println("No initial state specified");
            return;
        }
        
        m_stack = new StateExecutionStack(m_default);
        m_stack.schedule();
    }

    @Override
    public void execute() {
        if (m_stack == null)
            return;
        
        // Check if stack is dirty
        if (m_stack.isDirty()) {

            // If the dirty stack already scheduled a new stack,
            // discard the current one and continue execution
            if (m_stack.getNextStack() != null) {
                m_stack = m_stack.getNextStack();
            } else if (m_stack.getNextPath() != null) {
                // Stack needs to be replaced with new stack
                // and new state ancestry

                String[] path = m_stack.getNextPath();

                StateCommand baseState = getState(path[0]);
                if (path.length > 1) {
                    List<StateCommand> nextStates = StateCommandManager.getInstance()
                        .getStatesOnPath(baseState, path, 1);

                    nextStates.add(0, baseState);
                    m_stack = new StateExecutionStack(nextStates);
                } else {
                    m_stack = new StateExecutionStack(baseState);
                }

                // Schedule new stack
                m_stack.schedule();
            } else {
                m_stack = null;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (m_stack != null) {
            m_stack.cancel();
            m_stack = null;
        }
    }

    /**
     * Add states to group.
     * 
     * @param commands The states to add.
     */
    public void addStates(StateCommand... commands) {
        for (StateCommand cmd : Set.of(commands)) {
            final String name = cmd.getStateName();
            StateFormat.validateName(name);

            if (m_states.containsKey(name)) {
                throw new IllegalArgumentException(
                    "Conflict between commands using the name '"
                        + name + "' between command '" + cmd.getName()
                            +  "' and '" + m_states.get(name).getName() + "'");
            }

            m_states.put(name, cmd);
        }
    }

    /**
     * Set the default state.
     * 
     * <p> The default state is the state that runs
     * when no active state is specified upon initialization
     * of the command.
     * 
     * <p> Under normal circumstances, the default state
     * acts like the initial state of a state machine.
     * 
     * @param name The name of the default state.
     * 
     * @throws UnknownStateException If no command with {@code name} exists. 
     */
    public void setDefaultState(String name) throws UnknownStateException {
        m_default = getState(name);
    }
    
    /**
     * Set the active state.
     * 
     * <p> The active state is the state that runs
     * while the state machine is executing.
     * 
     * @param name The name of the default state.
     * 
     * @throws UnknownStateException If no command with {@code name} exists. 
     */
    public boolean setActiveState(String name) {
        if (m_stack == null)
            return false;
            
        StateCommand state = getState(name);

        // Interrupt the active stack
        m_stack.cancel();

        // Start new stack
        m_stack = new StateExecutionStack(state);
        m_stack.schedule();

        return true;
    }

    public StateCommand getState(String name) throws UnknownStateException {
        StateCommand command = m_states.get(name);
        if (command == null)
            throw new UnknownStateException("Command state '" + name + "' does not exist.");
        return command;
    }

    public Map<String, StateCommand> getStates() {
        return Collections.unmodifiableMap(m_states);
    }

    @Override
    public boolean isFinished() {
        return m_states == null;
    }
}
