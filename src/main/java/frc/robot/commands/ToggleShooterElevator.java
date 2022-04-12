package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.base.Property;


public class ToggleShooterElevator extends CommandBase {    
    private final Property<Boolean> climberActive;

    public ToggleShooterElevator(Property<Boolean> climberActive, Subsystem... requirements) {
        addRequirements(requirements);

        this.climberActive = climberActive;
    }

    @Override
    public void initialize() {
        climberActive.set(!climberActive.get());
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        // return turret.isTargetReached();
        return true;
    }
}