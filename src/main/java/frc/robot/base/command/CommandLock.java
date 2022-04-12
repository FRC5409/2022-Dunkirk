package frc.robot.base.command;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import edu.wpi.first.wpilibj2.command.Command;

public class CommandLock {
    private Command m_owner;
    
    public CommandLock() {
        m_owner = null;
    }

    public boolean acquire(@NotNull Command owner) {
        if (m_owner == null) {
            m_owner = owner;
            return true;
        } else
            return m_owner.equals(owner);
    }

    public boolean release(@NotNull Command owner) {
        if (m_owner == null)
            return true;
        else if (m_owner.equals(owner)) {
            m_owner = null;
            return true;
        } else {
            return false;
        }
    }

    @Nullable
    public Command getOwner() {
        return m_owner;
    }

    public boolean isAcquired() {
        return m_owner != null;
    }

    public boolean isOwner(Command owner) {
        return (m_owner == null) ? false : m_owner.equals(owner);
    }
}
