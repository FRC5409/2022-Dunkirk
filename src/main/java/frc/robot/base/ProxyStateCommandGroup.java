package frc.robot.base;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

import org.jetbrains.annotations.Nullable;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * <h2> ProxyStateCommandGroup </h2>
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
    protected StateCommand m_default;
    protected StateCommand m_active;

    /**
     * Construct an empty {@link ProxyStateCommandGroup}.
     */
    public ProxyStateCommandGroup() {
        m_active = null;
        m_default = null;
        m_states = new HashMap<>();
    }
    
    /**
     * Construct a {@link ProxyStateCommandGroup} with several states.
     * 
     * @param commands The states included in the group.
     */
    public ProxyStateCommandGroup(StateCommand... commands) {
        this();
        addCommands(commands);
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
        addCommands(commands);
        m_default = getCommand(defaultStateName);
    }

    @Override
    public void initialize() {
        // Check if state was externally set before command execution
        if (m_active == null) {
            // Use default command if specified
            if (m_default == null) {
                System.err.println("No initial state specified");
                return;
            }
            
            m_active = m_default;
            System.out.println("Starting with state " + m_active.getStateName());
        }

        m_active.schedule();
    }

    @Override
    public void execute() {
        if (m_active == null)
            return;
        
        if (!m_active.isScheduled()) {
            String next = m_active.getNextState();
            m_active.reset();

            if (next != null) {
                m_active = getCommand(next);
                System.out.println("Moving to state " + next);
                m_active.schedule();
            } else 
                m_active = null;
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted && m_active != null) {
            m_active.cancel();
            m_active = null;
        }
    }

    /**
     * Add states to group.
     * 
     * @param commands The states to add.
     */
    public void addCommands(StateCommand... commands) {
        for (StateCommand cmd : Set.of(commands)) {
            final String name = cmd.getStateName();
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
        m_default = getCommand(name);
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
        if (m_active == null)
            return false;
            
        StateCommand command = getCommand(name);

        // Interrupt the active command
        m_active.cancel();

        // Start new (or previously running) command
        command.schedule();
        m_active = command;

        return true;
    }

    @Nullable
    public String getActiveState() {
        if (m_active == null)
            return null;
        return m_active.getStateName();
    }

    public StateCommand getCommand(String name) throws UnknownStateException {
        StateCommand command = m_states.get(name);
        if (command == null)
            throw new UnknownStateException("Command state '" + name + "' does not exist.");
        return command;
    }

    public Map<String, StateCommand> getCommands() {
        return Collections.unmodifiableMap(m_states);
    }

    @Override
    public boolean isFinished() {
        return m_active == null;
    }
}
