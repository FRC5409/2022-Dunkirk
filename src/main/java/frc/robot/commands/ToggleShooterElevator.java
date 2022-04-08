package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.base.ValueProperty;


public class ToggleShooterElevator extends CommandBase {    
    private final ValueProperty<Boolean> climberActive;
    private final Subsystem[] subsystems;
    private ShooterTurret turret;


    public ToggleShooterElevator(ValueProperty<Boolean> climberActive, Subsystem... _subsystems) {
        this.climberActive = climberActive;
        subsystems = _subsystems;
        addRequirements(_subsystems);
    }

    @Override
    public void initialize() {
        for (Subsystem sys : subsystems) {
            if (sys instanceof ShooterTurret) {
                turret = (ShooterTurret)sys;

                turret.enable();

                turret.setRotationTarget(climberActive.get() ? 0.0 : 90.0);                
                break;
            }
        }

        climberActive.set(!climberActive.get());
        // SmartDashboard.putBoolean("Climber active", climberActive.get());
    }

    @Override
    public boolean isFinished() {
        return turret.isTargetReached();
    }
}
