package frc.robot.base.command;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class WrapperCommand extends CommandBase {
    private Command m_command;
    private boolean m_active;

    public WrapperCommand() {
        m_command = null;
    }

    public WrapperCommand(Command command) {
        setCommand(command);
    }

    @Override
    public void initialize() {
        if (m_command != null)
            m_command.initialize();
    }

    @Override
    public void execute() {
        if (m_command != null)
            m_command.execute();
    }

    @Override
    public void end(boolean interrupted) {
        if (m_command != null)
            m_command.end(interrupted);
    }

    public WrapperCommand setCommand(Command command) {
        if (m_active)
            throw new RuntimeException("Cannot set command while active.");
        
        m_command = command;
        m_requirements = command.getRequirements();

        return this;
    }

    @Override
    public boolean isFinished() {
        return (m_command == null) ? true : m_command.isFinished();
    }
}
