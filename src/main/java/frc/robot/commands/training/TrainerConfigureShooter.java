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
    private final Limelight limelight;
    private final ShooterMode target;
    private final ShooterTurret turret;
    private final TrainerContext context;
    private final TrainerDashboard dashboard;
    private final Property<ShooterConfiguration> property;

    public TrainerConfigureShooter(
        ShooterTurret turret,
        Limelight limelight,
        TrainerContext context,
        TrainerDashboard dashboard,
        Property<ShooterConfiguration> property,
        ShooterMode target
    ) {
        this.property  = property;
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

        context.setMode(target);
        
        ShooterConfiguration configuration = Constants.Shooter.CONFIGURATIONS.get(target);

        property.set(configuration);
        limelight.setPipelineIndex(configuration.getPipeline().id());

        switch (target) {
            case kNear: {
                turret.hoodDownPosition();
                break;
            }
            case kFar: {
                turret.hoodUpPosition();
                break;
            }
        }

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
