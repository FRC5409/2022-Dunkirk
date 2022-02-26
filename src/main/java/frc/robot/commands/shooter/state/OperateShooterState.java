package frc.robot.commands.shooter.state;

import org.jetbrains.annotations.NotNull;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.base.StateCommandBase;
import frc.robot.base.shooter.ShooterModel;
import frc.robot.base.shooter.SweepDirection;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.TargetType;
import frc.robot.subsystems.shooter.ShooterFlywheel;
import frc.robot.subsystems.shooter.ShooterTurret;
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
public class OperateShooterState extends StateCommandBase {
    private final ShooterFlywheel flywheel;
    private final ShooterTurret turret;
    private final Limelight limelight;
    private final Indexer indexer;
    
    private final ShooterModel model;
    private boolean linear;
    
    public OperateShooterState(
        Limelight limelight,
        ShooterTurret turret,
        ShooterFlywheel flywheel,
        Indexer indexer,
        ShooterModel model
    ) {
        this.limelight = limelight;
        this.flywheel = flywheel;
        this.indexer = indexer;
        this.turret = turret;
        this.model = model;

        addRequirements(limelight, turret, flywheel, indexer);
    }

    public OperateShooterState(
        Limelight limelight,
        ShooterTurret turret,
        ShooterFlywheel flywheel,
        Indexer indexer,
        ShooterModel model, 
        boolean linear
    ) {
        this.limelight = limelight;
        this.flywheel = flywheel;
        this.indexer = indexer;
        this.turret = turret;
        this.model = model;
        this.linear = linear;

        addRequirements(limelight, turret, flywheel, indexer);
    }



    @Override
    public void initialize() {
        if (!Toggleable.isEnabled(limelight, turret, flywheel, indexer))
            throw new RuntimeException("Cannot operate shooter when requirements are not enabled.");

        flywheel.spinFeeder(4500);
    }

    @Override
    public void execute() {
        Vector2 target = limelight.getTarget();

        double distance = Constants.Vision.DISTANCE_FUNCTION.calculate(target.y);
        double velocity;
        if(linear){
            System.out.println("Linear calculation");
            velocity = model.calculateLinear(distance);
        } else {
            velocity = model.calculate(distance);   
        }

        // Set flywheel to estimated veloctity
        flywheel.setVelocity(velocity);

        // Continue aligning shooter
        if (Math.abs(target.x) > Constants.Vision.ALIGNMENT_THRESHOLD)
            turret.setRotationTarget(turret.getRotation() + target.x* Constants.Vision.ROTATION_P);

        if (turret.isTargetReached() && flywheel.isTargetReached()) {
            indexer.spinIndexer(1);
        } /*else {
            indexer.moveIndexerMotor(0);
        }*/

        SmartDashboard.putNumber("Velocity Prediction", velocity);
        SmartDashboard.putNumber("Active Velocity", flywheel.getVelocity());
        
        SmartDashboard.putNumber("Distance Prediction (ft)", distance);
        SmartDashboard.putNumber("Aligninment Offset", target.x);

    }

    @Override
    public void end(boolean interrupted) {
        flywheel.setVelocity(0);
        indexer.stopIndexer();
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