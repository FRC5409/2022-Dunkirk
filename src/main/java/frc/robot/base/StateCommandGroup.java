package frc.robot.base;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

import org.jetbrains.annotations.Nullable;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * <h2> StateCommandGroup </h2>
 * A command-based state machine representing a series of tasks to be performed by the robot
 * in an independant execution order.
 * 
 * <p> {@link edu.wpi.first.wpilibj2.command.Command StateCommand}s are an extension of
 * {@link edu.wpi.first.wpilibj2.command.Command Command}s that represent a task to be
 * performed by the robot, as opposed to a complete action performed by the robot. This provides
 * States with the flexibility of Commands, and the portability and seperation
 * of individual States.
 * 
 * @author Keith Davies
 * @see StateCommand
 */
public abstract class StateCommandGroup extends CommandBase {
    protected Map<String, StateCommand> m_states;
    protected StateCommand m_default;
    protected StateCommand m_active;

    /**
     * Construct an empty {@link StateCommandGroup}.
     */
    public StateCommandGroup() {
        m_active = null;
        m_default = null;
        m_states = new HashMap<>();
    }
    
    /**
     * Construct a {@link StateCommandGroup} with several states.
     * 
     * @param commands The states included in the group.
     */
    public StateCommandGroup(StateCommand... commands) {
        this();
        addCommands(commands);
    }

    
    /**
     * Construct a {@link StateCommandGroup} with several states,
     * as well as a default state.
     * 
     * @param commands         The states included in the group.
     * @param defaultStateName The default state name.
     */
    public StateCommandGroup(String defaultStateName, StateCommand... commands) {
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
            System.out.println ("Starting with state " + m_active.getStateName());
        }

        m_active.initialize();
    }

    @Override
    public void execute() {
        if (m_active == null)
            return;

        // A state can only switch when finished
        if (m_active.isFinished()) {
            m_active.end(false);

            String next = m_active.getNextState();
            m_active.reset();

            if (next != null) {
                m_active = getCommand(next);
                System.out.println("Moving to state " + next);

                m_active.initialize();
            } else 
                m_active = null;
        } else
            m_active.execute();
    }

    @Override
    public void end(boolean interrupted) {
        if (m_active == null)
            return;

        m_active.end(interrupted);
        m_active = null;
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

            m_requirements.addAll(cmd.getRequirements());
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
        StateCommand command = getCommand(name);

        // Interrupt the active command
        m_active.end(true);

        // Start new (or previously running) command
        m_active = command;
        m_active.initialize();

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
