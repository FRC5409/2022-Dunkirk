package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DelayedCancelCommand extends CommandBase {
    private final Command target;
    private final double delay;

    private double last;

    public DelayedCancelCommand(double delay, Command target) {
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
        System.out.println("Cancelled new target after delay of " + delay + "s");
        target.cancel();
    }

    @Override
    public boolean isFinished() {
        return (Timer.getFPGATimestamp() - last) > delay;
    }
}
