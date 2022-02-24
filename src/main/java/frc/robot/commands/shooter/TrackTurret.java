package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.shooter.*;
import frc.robot.utils.*;

/**
 * This command runs the turret flywheel at a speed
 * porportional to the distance of the turret from the
 * shooting target and operates the indexer once the rpm
 * reaches it's setpoint.
 * 
 * @author Akil Pathiranage, Keith Davies
 */
public final class TrackTurret extends CommandBase {
    private final ShooterTurret turret;
    private final Limelight    limelight;

    public TrackTurret(ShooterTurret turret, Limelight limelight) {
        this.turret = turret;
        this.limelight = limelight;

        addRequirements(turret, limelight);
    }

    @Override
    public void initialize() {
        turret.enable();

        limelight.enable();
        limelight.setLedMode(Limelight.LedMode.kModeOn);
    }

    @Override
    public void execute() {
        Vector2 target = limelight.getTarget();
        turret.setRotationTarget(turret.getRotation()+target.x);
    }

    @Override
    public void end(boolean interrupted) {
        limelight.disable();
        turret.disable();
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}