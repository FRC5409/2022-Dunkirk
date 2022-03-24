package frc.robot.base.command;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public abstract class TimedCommand extends CommandBase {
    private double m_start;

    @Override
    public void initialize() {
        m_start = Timer.getFPGATimestamp();
    }

    public double getStartTime() {
        return m_start;
    }

    public double getElapsedTime() {
        return Timer.getFPGATimestamp() - m_start;
    }
}
