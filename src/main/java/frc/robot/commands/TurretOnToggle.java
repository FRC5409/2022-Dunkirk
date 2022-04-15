package frc.robot.commands;
import frc.robot.base.Property;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.ShooterTurret;
import frc.robot.subsystems.shooter.ShooterTurret.ReferenceType;

public class TurretOnToggle extends CommandBase {
    private final Property<Boolean> climberActive;
    private final ShooterTurret turret;

    public TurretOnToggle(Property<Boolean> _climberActive, ShooterTurret _turret) {
        turret = _turret;
        climberActive = _climberActive;

        addRequirements(_turret);
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
