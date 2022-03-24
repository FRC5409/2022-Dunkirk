package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DelayedScheduleCommand extends CommandBase {
    private final Set<Command> commands;
    private final double delay;
    private double last;

    public DelayedScheduleCommand(double delay, Command... commands) {
        this.commands = Set.of(commands);
        this.delay = delay;
    }
    
    @Override
    public void initialize() {
        last = Timer.getFPGATimestamp();
    }

    @Override
    public void end(boolean interrupted) {
        for (Command cmd : commands) {
            cmd.schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return (Timer.getFPGATimestamp() - last) > delay;
    }
}
