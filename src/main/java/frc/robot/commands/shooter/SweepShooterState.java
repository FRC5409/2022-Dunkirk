package frc.robot.commands.shooter;

import org.jetbrains.annotations.NotNull;

import frc.robot.Constants;
import frc.robot.base.TimedStateCommand;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.TargetType;
import frc.robot.subsystems.shooter.ShooterTurret;

// TODO update doc

/**
 * <h3>Operates the turret in the "Sweeping" state.</h>
 * 
 * <p>In this state, the turret sweeps about it's axis
 * smoothly using a scaled cosine function. If at any point the
 * limelight detects a target, it switches over to the {@code kShooting}
 * state. If there is no target after "x" amount of sweeps, (See Constants)
 * the command cancels itself. The reason for this is due to the fact
 * that the limelight's led's may not be on for prolonged periods of time.</p>
 */
public class SweepShooterState extends TimedStateCommand {
    private final ShooterTurret turret;
    private final Limelight limelight;

    private double offsetTime;
    private boolean done;

    public SweepShooterState(Limelight limelight, ShooterTurret turret) {
        this.limelight = limelight;
        this.turret = turret;

        addRequirements(limelight, turret);
    }

    @Override
    public void initialize() {
        super.initialize();

        offsetTime = Constants.Shooter.SHOOTER_SWEEP_INVERSE.calculate(turret.getRotation());
        done = false;
    }

    @Override
    public void execute() {
        if (limelight.hasTarget() && limelight.getTargetType() == TargetType.kHub) {
            next("frc.robot.shooter:operate");
            done = true;
        } else if (getElapsedTime() / Constants.Shooter.SHOOTER_SWEEP_PERIOD > Constants.Shooter.SHOOTER_MAX_SWEEEP) {
            done = true;
        } else {
            turret.setRotationTarget(
                Constants.Shooter.SHOOTER_SWEEP_FUNCTION.calculate(getElapsedTime() + offsetTime)
            );
        } 
    }

    @Override
    public boolean isFinished() {
        return done;
    }

    @Override
    public @NotNull String getStateName() {
        return "frc.robot.shooter:sweep";
    }
}
