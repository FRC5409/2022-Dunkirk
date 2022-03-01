package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.base.Property;
import frc.robot.base.shooter.ShooterConfiguration;
import frc.robot.base.shooter.ShooterMode;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.shooter.ShooterTurret;

public class ConfigureShooter extends CommandBase {
    private final Property<ShooterConfiguration> configuration;
    private final ShooterMode target;

    private final ShooterTurret turret;
    private final Limelight limelight;

    public ConfigureShooter(
        ShooterTurret turret,
        Limelight limelight,
        Property<ShooterConfiguration> configuration,
        ShooterMode target
    ) {
        this.configuration = configuration;
        this.limelight = limelight;
        this.target = target;
        this.turret = turret;
    }

    @Override
    public void initialize() {
        turret.enable();
        limelight.enable();

        configuration.set(
            Constants.Shooter.CONFIGURATIONS.get(target)
        );
        
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
