package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * This is bad, do not use.
 */
public class RequireCommand extends CommandBase {
    public RequireCommand(Subsystem... requirements) {
        addRequirements(requirements);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
