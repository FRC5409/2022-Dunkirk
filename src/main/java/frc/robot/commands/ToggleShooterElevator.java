package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.base.ValueProperty;


public class ToggleShooterElevator extends CommandBase {    
    private final ValueProperty<Boolean> climberActive;


    public ToggleShooterElevator(ValueProperty<Boolean> climberActive, Subsystem... subsystems) {
        this.climberActive = climberActive;
        addRequirements(subsystems);
    }

    @Override
    public void execute() {
        climberActive.set(!climberActive.get());
        SmartDashboard.putBoolean("Climber active", climberActive.get());
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
