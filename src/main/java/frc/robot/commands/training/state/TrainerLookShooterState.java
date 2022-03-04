package frc.robot.commands.training.state;

import org.jetbrains.annotations.NotNull;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.base.StateCommandBase;
import frc.robot.base.shooter.ShooterModel;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.TargetType;
import frc.robot.subsystems.shooter.ShooterTurret;
import frc.robot.training.TrainerDashboard;
import frc.robot.training.Setpoint;
import frc.robot.training.TrainerContext;
import frc.robot.utils.Vector2;

// TODO update doc

/**
 * <h3>Operates the turret in the "Shooting" state.</h>
 * 
 * <p>In this state, the turret runs it's flywheel at a speed 
 * porportional to the distance of the turret from the outer port
 * and aligns the turret's rotation axis to the target.
 * Once both systems have reached their respective targets,
 * the indexer triggers, feeding powercells into the turret.</p>
 */
public class TrainerLookShooterState extends StateCommandBase {
    private final ShooterTurret turret;
    private final Limelight limelight;
    private final TrainerDashboard dashboard;
    private final TrainerContext context;

    public TrainerLookShooterState(
        Limelight limelight,
        ShooterTurret turret,
        TrainerDashboard dashboard,
        TrainerContext context
    ) {
        this.limelight = limelight;
        this.turret = turret;
        this.dashboard = dashboard;
        this.context = context;

        addRequirements(limelight, turret);
    }

    @Override
    public void execute() {
        Vector2 target = limelight.getTarget();

        ShooterModel model = context.getModel();

        double distance = model.distance(target.y);
        double speed = model.calculate(distance);

        context.setDistance(distance);
        context.setSetpoint(
            new Setpoint(speed, Constants.Shooter.SPEED_RANGE)
        );

        // Continue aligning shooter
        if (Math.abs(target.x) > Constants.Vision.ALIGNMENT_THRESHOLD)
            turret.setRotationTarget(turret.getRotation() + target.x * Constants.Vision.ROTATION_P);
            
        if(Constants.kConfig.DEBUG) SmartDashboard.putNumber("Aligninment Offset", target.x);
        
        dashboard.update();
    }

    @Override
    public boolean isFinished() {
        return !(limelight.hasTarget() && limelight.getTargetType() == TargetType.kHub);
    }

    @Override
    public @NotNull String getStateName() {
        return "frc.robot.shooter:operate";
    }
}
