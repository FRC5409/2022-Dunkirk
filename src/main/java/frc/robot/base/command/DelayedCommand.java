package frc.robot.base.command;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DelayedCommand extends CommandBase {
    private final Command target;
    private final double delay;

    private double last;

    public DelayedCommand(double delay, Command target) {
        this.target = target;
        this.delay = delay;

        last = 0;
    }
    
    @Override
    public void initialize() {
        last = Timer.getFPGATimestamp();
    }

    @Override
    public void end(boolean interrupted) {
        target.schedule();
    }

    @Override
    public boolean isFinished() {
        return (Timer.getFPGATimestamp() - last) > delay;
    }
}
