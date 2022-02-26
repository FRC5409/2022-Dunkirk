package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ToggleShooterElevator extends CommandBase {
    private final Climber climber;

    public ToggleShooterElevator(Climber climber) {
        this.climber = climber;

        addRequirements(climber);
    }

    @Override
    public void initialize() {
        if (climber.getActive()) {
            climber.setActive(false);
            // Shooter activate
        } else {
            climber.setActive(true);
            // Shooter deactivate
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
