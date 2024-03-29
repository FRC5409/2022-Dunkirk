package frc.robot.base.command;

import edu.wpi.first.wpilibj.Timer;

public abstract class TimedStateCommand extends StateBase {
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
