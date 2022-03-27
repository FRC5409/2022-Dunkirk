package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.base.Property;
import frc.robot.base.shooter.ShooterConfiguration;
import frc.robot.base.shooter.ShooterMode;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.shooter.ShooterTurret;

/**
 * Command for configuring the shooter. Whenever you want
 * to change a part of the configuration for the shooter, use this command. 
 */
public class ConfigureShooter extends CommandBase {
    private final Property<ShooterConfiguration> configuration;
    private final ShooterMode target;

    private final ShooterTurret turret;
    private final Limelight limelight;

    /**
     * 
     * @param turret
     * @param limelight
     * @param configuration Configuration to apply the new config to, pass in the current config you are using for the shooter. 
     * @param target
     */
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

        addRequirements(turret, limelight);
    }

    @Override
    public void initialize() {
        turret.enable();
        limelight.enable();

        ShooterConfiguration config = Constants.Shooter.CONFIGURATIONS.get(target);
        configuration.set(config);

        limelight.setPipelineIndex(config.getPipeline().id());
        turret.setHoodPosition(config.getHoodPosition());
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
