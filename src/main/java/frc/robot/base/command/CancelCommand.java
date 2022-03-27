package frc.robot.base.command;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CancelCommand extends CommandBase {
    private final Command target;

    public CancelCommand(Command target) {
        this.target = target;
    }
    
    @Override
    public void initialize() {
        target.cancel();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
