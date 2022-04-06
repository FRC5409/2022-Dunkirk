package frc.robot.base.command;

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
 * @see State
 */
public abstract class StateGroupCommand extends CommandBase {
    protected Map<String, State> m_states;
    protected State m_default;
    protected State m_active;

    /**
     * Construct an empty {@link StateGroupCommand}.
     */
    public StateGroupCommand() {
        m_active = null;
        m_default = null;
        m_states = new HashMap<>();
    }
    
    /**
     * Construct a {@link StateGroupCommand} with several states.
     * 
     * @param commands The states included in the group.
     */
    public StateGroupCommand(State... commands) {
        this();
        addCommands(commands);
    }

    
    /**
     * Construct a {@link StateGroupCommand} with several states,
     * as well as a default state.
     * 
     * @param commands         The states included in the group.
     * @param defaultStateName The default state name.
     */
    public StateGroupCommand(String defaultStateName, State... commands) {
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
            System.out.println ("Starting with state " + m_active.getName());
        }

        m_active.initialize();
    }

    @Override
    public void execute() {
        if (m_active == null)
            return;

        // A state can only switch when finished
        if (m_active.isFinished()) {
            String next = m_active.getNextState();
            
            if (next != null) {
                m_active.end(InterruptType.kTransition);
                m_active.reset();

                m_active = getCommand(next);
                System.out.println("Moving to state " + next);

                m_active.initialize();
            } else {
                m_active.end(InterruptType.kFinish);
                m_active = null;
            }
        } else
            m_active.execute();
    }

    @Override
    public void end(boolean interrupted) {
        if (m_active == null)
            return;

        // We don't need to check the interrupted flag
        // because, under normal circumstances, m_active
        // will be null when the state command group ends
        m_active.end(InterruptType.kCancel);
        m_active = null;
    }

    /**
     * Add states to group.
     * 
     * @param commands The states to add.
     */
    public void addCommands(State... commands) {
        for (State cmd : Set.of(commands)) {
            final String name = cmd.getName();
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
        if (m_active == null)
            return false;

        State command = getCommand(name);

        // Interrupt the active command
        m_active.end(InterruptType.kCancel);

        // Start new (or previously running) command
        command.initialize();

        m_active = command;

        return true;
    }

    @Nullable
    public String getActiveState() {
        if (m_active == null)
            return null;
        return m_active.getName();
    }

    public State getCommand(String name) throws UnknownStateException {
        State command = m_states.get(name);
        if (command == null)
            throw new UnknownStateException("Command state '" + name + "' does not exist.");
        return command;
    }

    public Map<String, State> getCommands() {
        return Collections.unmodifiableMap(m_states);
    }

    @Override
    public boolean isFinished() {
        return m_active == null;
    }
}
