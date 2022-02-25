package frc.robot.base;

import org.jetbrains.annotations.NotNull;

import edu.wpi.first.wpilibj2.command.Command;

public class StateProxyCommand extends StateCommandBase {
    protected Command m_command;
    protected String  m_stateName;

    public StateProxyCommand(String name, Command command) {
        m_command = command;
        m_stateName = name;
    }

    @Override
    public void initialize() {
        m_command.initialize();
    }

    @Override
    public void execute() {
        m_command.execute();
    }

    @Override
    public void end(boolean interrupted) {
        m_command.end(interrupted);
    }

    @Override
    public boolean isFinished() {
       return m_command.isFinished();
    }

    @Override
    public @NotNull String getStateName() {
        return m_stateName;
    }

    public Command getCommand() {
        return m_command;
    }
}
