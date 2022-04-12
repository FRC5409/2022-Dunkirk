package frc.robot.base.command;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
 
public class SynchronizedCommand extends CommandBase {
    private final Set<CommandLock> m_locks;
    private final List<Command> m_commands;

    private boolean m_active;
    private boolean m_finished;

    public SynchronizedCommand() {
        m_locks = new HashSet<>();
        m_commands = new ArrayList<>();
    }

    public SynchronizedCommand(CommandLock... locks) {
        this();
        addLocks(locks);
    }

    public SynchronizedCommand(CommandLock lock, Command... commands) {
        this();
        addCommands(commands);
        addLocks(lock);
    }

    public SynchronizedCommand(Set<CommandLock> locks, Command... commands) {
        this();
        m_locks.addAll(locks);
        addCommands(commands);
    }

    @Override
    public void initialize() {
        m_active = acquireLocks();
        m_finished = false;

        // If we acquired the lock, schedule the commands
        if (m_active) {
            for (Command command : m_commands)
                command.schedule();
        }
    }

    @Override
    public void execute() {
        // Continue attempting to acquire lock
        if (m_active) {
            m_finished = true;
            for (Command command : m_commands)
                m_finished &= !command.isScheduled();
        } else {
            m_active = acquireLocks();

            // If we acquired the lock, schedule the commands
            if (m_active) {
                for (Command command : m_commands)
                    command.schedule();
            }
        } 
    }

    @Override
    public void end(boolean interrupted) {
        if (m_active) {
            for (Command command : m_commands) {
                if (command.isScheduled())
                    command.cancel();
            }
        }

        for (CommandLock lock : m_locks)
            lock.release(this);
    }

    public SynchronizedCommand addCommands(Command... commands) {
        Set<Command> ungrouped = Set.of(commands);
        if (!Collections.disjoint(m_commands, ungrouped))
            throw new IllegalArgumentException("Cannot re-add commands.");

        m_commands.addAll(ungrouped);

        return this;
    }

    public SynchronizedCommand addLocks(CommandLock... locks) {
        m_locks.addAll(Set.of(locks));
        return this;
    }

    @Override
    public boolean isFinished() {
        return m_finished;
    }

    private boolean acquireLocks() {
        boolean acquired = true;
        for (CommandLock lock : m_locks)
            acquired &= lock.acquire(this);
        return acquired;
    }
}
