package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.base.Property;
import frc.robot.base.shooter.ShooterMode;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.shooter.ShooterTurret;

public class ConfigureShooterMode extends CommandBase {
    private final Property<ShooterMode> property;
    private final ShooterMode           target;
    private final ShooterTurret         turret;
    private final Limelight             limelight;

    public ConfigureShooterMode(
        ShooterTurret turret,
        Limelight limelight,
        Property<ShooterMode> property,
        ShooterMode target
    ) {
        this.property  = property;
        this.target    = target;
        this.turret    = turret;
        this.limelight = limelight;
    }

    @Override
    public void initialize() {
        turret.enable();
        limelight.enable();

        property.set(target);
        
        switch (target) {
            case kNear: {
                turret.hoodDownPosition();
                limelight.setPipelineIndex(1);
                break;
            }
            case kFar: {
                turret.hoodUpPosition();
                limelight.setPipelineIndex(2);
                break;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        limelight.disable();
        turret.disable();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
