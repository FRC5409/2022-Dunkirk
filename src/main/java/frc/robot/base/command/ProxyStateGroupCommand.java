package frc.robot.base.command;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * <h2> ProxyStateCommandGroupTest </h2>
 * A command-based state machine representing a series of tasks to be performed by the robot
 * in an independant execution order.
 * 
 * <p>When the {@link ProxyStateGroupCommand} executes, it schedules states without obtaining
 * their requirements, as opposed to the typical behaviour of a {@link StateGroupCommand}.</p>
 * 
 * <p>Suppose a state group with states [A, B, C, D]. If state 'A' requires a given subsystem,
 * the state will only obtain ownership of the subsytem upon its individual execution, as opposed
 * to the execution of the group.</p>
 * 
 * @author Keith Davies
 * @see StateCommand, StateCommandGroup
 */
public abstract class ProxyStateGroupCommand extends CommandBase {
    protected StateExecutionStack m_stack;
    protected Map<String, State> m_states;
    protected List<State> m_defaults;
    protected boolean m_active;

    /**
     * Construct an empty {@link ProxyStateGroupCommand}.
     */
    public ProxyStateGroupCommand() {
        m_defaults = null;
        m_states = new HashMap<>();
        m_stack = null;
    }
    
    /**
     * Construct a {@link ProxyStateGroupCommand} with several states.
     * 
     * @param commands The states included in the group.
     */
    public ProxyStateGroupCommand(State... commands) {
        this();
        addStates(commands);
    }

    
    /**
     * Construct a {@link ProxyStateGroupCommand} with several states,
     * as well as a default state.
     * 
     * @param commands         The states included in the group.
     * @param defaultStateName The default state name.
     */
    public ProxyStateGroupCommand(String defaultStatePath, State... commands) {
        this();
        addStates(commands);
        setDefaultState(defaultStatePath);
    }

    @Override
    public void initialize() {
        if (m_stack == null) {
            if (m_defaults == null) {
                System.err.println("No initial state specified");
                return;
            } else {
                m_stack = new StateExecutionStack(m_defaults);
            }
        }
        
        m_stack.schedule();
        m_active = true;
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

                State baseState = getState(path[0]);
                if (path.length > 1) {
                    List<State> nextStates = StateManager.getInstance()
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

        m_active = false;
    }

    /**
     * Add states to group.
     * 
     * @param commands The states to add.
     */
    public void addStates(State... states) {
        for (State cmd : Set.of(states)) {
            final String name = cmd.getName();
            StateNameFormat.assertName(name);

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
    public void setDefaultState(String path) throws UnknownStateException {
        m_defaults = getStates(StateNameFormat.parsePath(path));
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
    public boolean setActiveState(String path) {
        StateExecutionStack stack = new StateExecutionStack(
            getStates(StateNameFormat.parsePath(path))
        );

        if (m_active) {
            // Interrupt the active stack
            m_stack.cancel();

            // Start new stack
            stack.schedule();
        }

        m_stack = stack;

        return true;
    }

    protected State getState(String name) throws UnknownStateException {
        State state = m_states.get(name);
        if (state == null)
            throw new UnknownStateException("Command state '" + name + "' does not exist.");
        return state;
    }

    protected List<State> getStates(String[] path) throws UnknownStateException {
        if (path.length == 0)
            throw new UnknownStateException("Cannot find state with empty path");

        State base = m_states.get(path[0]);
        if (base == null)
            throw new UnknownStateException("Command state '" + path[0] + "' does not exist.");
    
        List<State> states;

        if (path.length > 1) {
            states = StateManager.getInstance()
                .getStatesOnPath(base, path, 1);
        } else {
            states = new ArrayList<>();
        }

        states.add(0, base);

        return states;
    }

    public Map<String, State> getStates() {
        return Collections.unmodifiableMap(m_states);
    }

    @Override
    public boolean isFinished() {
        return m_stack == null;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringArrayProperty("Active States", () -> {
            if (m_stack == null)
                return new String[]{};
            return m_stack.getStates();
        }, null);
    }
}
