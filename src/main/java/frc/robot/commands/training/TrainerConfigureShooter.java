package frc.robot.commands.training;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.base.Property;
import frc.robot.base.shooter.ShooterConfiguration;
import frc.robot.base.shooter.ShooterMode;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.shooter.ShooterTurret;
import frc.robot.training.TrainerContext;
import frc.robot.training.TrainerDashboard;

public class TrainerConfigureShooter extends CommandBase {
    private final Property<ShooterConfiguration> configuration;

    private final Limelight limelight;
    private final ShooterMode target;
    private final ShooterTurret turret;
    private final TrainerContext context;
    private final TrainerDashboard dashboard;

    public TrainerConfigureShooter(
        ShooterTurret turret,
        Limelight limelight,
        TrainerContext context,
        TrainerDashboard dashboard,
        Property<ShooterConfiguration> configuration,
        ShooterMode target
    ) {
        this.configuration  = configuration;
        this.target    = target;
        this.turret    = turret;
        this.limelight = limelight;
        this.context   = context;
        this.dashboard = dashboard;
    }

    @Override
    public void initialize() {
        turret.enable();
        limelight.enable();

        ShooterConfiguration config = Constants.Shooter.CONFIGURATIONS.get(target);
        configuration.set(config);

        limelight.setPipelineIndex(config.getPipeline().id());
        switch (config.getHoodPosition()) {
            case kDown: {
                turret.hoodDownPosition();
                break;
            }
            case kUp: {
                turret.hoodUpPosition();
                break;
            }
        }

        context.setMode(target);

        dashboard.update();
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
