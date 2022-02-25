package frc.robot.commands.shooter.state;

import org.jetbrains.annotations.NotNull;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.base.TimedStateCommand;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.shooter.ShooterTurret;
import frc.robot.utils.Vector2;

// TODO update doc
public class AlignShooterState extends TimedStateCommand {
    private final ShooterTurret turret;
    private final Limelight limelight;
    private boolean done;

    public AlignShooterState(Limelight limelight, ShooterTurret turret) {
        this.limelight = limelight;
        this.turret = turret;

        addRequirements(limelight, turret);
    }

    @Override
    public void initialize() {
        super.initialize();
        
        done = false;
    }

    @Override
    public void execute() {
        Vector2 target = limelight.getTarget();

        if (Math.abs(target.x) < Constants.Vision.ALIGNMENT_THRESHOLD) {
            next("frc.robot.shooter:operate");
            done = true;
        } else if (getElapsedTime() > Constants.Shooter.ALIGNMENT_MAX_TIME)
            done = true;
        else
            turret.setRotationTarget(turret.getRotation() + target.x);
        
        SmartDashboard.putNumber("Alignment Offset", target.x);
    }

    @Override
    public boolean isFinished() {
        return done;
    }

    @Override
    public @NotNull String getStateName() {
        return "frc.robot.shooter:align";
    }
}
