package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.base.Property;
import frc.robot.subsystems.shooter.ShooterTurret;
import frc.robot.subsystems.shooter.ShooterTurret.ReferenceType;


public class ToggleShooterElevator extends CommandBase {    
    private final Property<Boolean> climberActive;
    private final ShooterTurret turret;

    public ToggleShooterElevator(Property<Boolean> climberActive, ShooterTurret turret, Subsystem... requirements) {
        addRequirements(turret);
        addRequirements(requirements);

        this.climberActive = climberActive;
        this.turret = turret;
    }

    @Override
    public void initialize() {
        turret.enable();
        turret.setReference(climberActive.get() ? 0.0 : 90.0, ReferenceType.kRotation);         

        climberActive.set(!climberActive.get());
    }

    @Override
    public void end(boolean interrupted) {
        turret.disable();
    }

    @Override
    public boolean isFinished() {
        return turret.isTargetReached();
    }
}
