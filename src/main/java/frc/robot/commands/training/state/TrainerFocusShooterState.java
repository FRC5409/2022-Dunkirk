package frc.robot.commands.training.state;

import org.jetbrains.annotations.NotNull;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.base.command.StateCommandBase;
import frc.robot.base.shooter.odometry.ShooterExecutionModel;
import frc.robot.base.shooter.odometry.SimpleShooterOdometry;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.TargetType;
import frc.robot.subsystems.shooter.ShooterTurret;
import frc.robot.training.TrainerDashboard;
import frc.robot.training.Setpoint;
import frc.robot.training.TrainerContext;
import frc.robot.utils.Toggleable;
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
public class TrainerFocusShooterState extends StateCommandBase {
    private final ShooterTurret turret;
    private final Limelight limelight;
    private final TrainerDashboard dashboard;
    private final TrainerContext context;

    private SimpleShooterOdometry odometry;
    private ShooterExecutionModel model;

    public TrainerFocusShooterState(
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
    public void initialize() {        
        if (!Toggleable.isEnabled(limelight, turret))
            throw new RuntimeException("Cannot operate shooter when requirements are not enabled.");

        // Initialize odometry
        odometry = new SimpleShooterOdometry(context.getOdometryModel());
        model = context.getExecutionModel();
    }

    @Override
    public void execute() {
        // Update odometry target
        if (limelight.getTargetType() == TargetType.kHub)
            odometry.update(limelight.getTargetPosition());

        Vector2 target = odometry.getTarget();

        // Continue aligning shooter
        if (Math.abs(target.x) > Constants.Vision.ALIGNMENT_THRESHOLD)
            turret.setRotationTarget(turret.getRotation() + target.x * Constants.Vision.ROTATION_P);

        context.setDistance(odometry.getDistance());
        
        context.setSetpoint(
            new Setpoint(
                model.calculate(odometry.getDistance()),
                Constants.Shooter.SPEED_RANGE
            )
        );
            
        SmartDashboard.putNumber("Alignment Offset", target.x);
        SmartDashboard.putData("Shooter Odometry", odometry);
        
        dashboard.update();
    }

    @Override
    public boolean isFinished() {
        return limelight.getTargetType() != TargetType.kHub;
    }

    @Override
    public @NotNull String getStateName() {
        return "frc.robot.shooter:operate";
    }
}
