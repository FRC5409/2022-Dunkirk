package frc.robot.base.command;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProxyScheduleCommand;

/**
 * A command that will only schedule another command after a specified
 * timeout from the previous execution period.
 */
public class DebounceScheduleCommand extends ProxyScheduleCommand {
    private final double m_timeout;
    private boolean m_active;
    private double m_last;

    public DebounceScheduleCommand(double timeout, Command... toSchedule) {
        super(toSchedule);

        m_timeout = timeout; 
        m_active = false;
        m_last = 0;
    }

    @Override
    public void initialize() {
        m_active = canExecute();
        if (m_active)
            super.initialize();
    }

    @Override
    public void execute() {
        if (m_active)
            super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        if (m_active) {
            super.end(interrupted);
            m_last = Timer.getFPGATimestamp();
        }
    }

    @Override
    public boolean isFinished() {
        if (m_active)
            return super.isFinished();
        return true;
    }

    public boolean canExecute() {
        return (Timer.getFPGATimestamp() - m_last) > m_timeout;
    }
}
