package frc.robot.base.command;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ProxySequentialCommandGroup extends CommandBase {
    protected List<Command> m_commands;
    protected int m_index;

    protected Command m_active;
    
    public ProxySequentialCommandGroup(Command... commands) {
        m_commands = new ArrayList<>();
        m_index = 0;
        m_commands.addAll(List.of(commands));
    }

    @Override
    public void initialize() {
        m_index = 0;

        m_active = getCommand(m_index);
        if (m_active != null)
            m_active.schedule();
        //System.out.println("Starting with command #" + m_index);
    }

    @Override
    public void execute() {
        if (m_active != null) {
            if (!m_active.isScheduled()) {
                m_index++;
                //System.out.println("Moving to next command #" + m_index);
                m_active = getCommand(m_index);
                if (m_active != null)
                    m_active.schedule();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (m_active != null)
            m_active.cancel();
    }

    public final void addCommands(Command... commands) {
        m_commands.addAll(List.of(commands));
    }

    @Override
    public boolean isFinished() {
        return m_active == null;
    }

    private Command getCommand(int index) {
        if (m_index >= m_commands.size())
            return null;
        return m_commands.get(index);
    }
}
