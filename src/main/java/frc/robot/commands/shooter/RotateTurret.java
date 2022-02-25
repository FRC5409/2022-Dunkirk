package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.*;

// TODO update doc

/**
 * This command runs the turret flywheel at a speed
 * porportional to the distance of the turret from the
 * shooting target and operates the indexer once the rpm
 * reaches it's setpoint.
 * 
 * @author Akil Pathiranage, Keith Davies
 */
public final class RotateTurret extends CommandBase {
    private final ShooterTurret turret;
    private final double        target;

    public RotateTurret(ShooterTurret turret, double target) {
        this.turret = turret;
        this.target = target;

        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.enable();
    }

    @Override
    public void execute() {
        turret.setRotationTarget(target);
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