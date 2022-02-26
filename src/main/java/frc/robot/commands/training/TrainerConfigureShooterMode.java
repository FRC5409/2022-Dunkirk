package frc.robot.commands.training;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.base.Property;
import frc.robot.base.shooter.ShooterMode;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.shooter.ShooterTurret;
import frc.robot.training.TrainerContext;
import frc.robot.training.TrainerDashboard;

public class TrainerConfigureShooterMode extends CommandBase {
    private final Property<ShooterMode> property;
    private final ShooterMode           target;
    private final ShooterTurret         turret;
    private final Limelight             limelight;
    private final TrainerContext        context;
    private final TrainerDashboard      dashboard;

    public TrainerConfigureShooterMode(
        ShooterTurret turret,
        Limelight limelight,
        TrainerContext context,
        TrainerDashboard dashboard,
        Property<ShooterMode> property,
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

        property.set(target);
        context.setMode(target);

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
