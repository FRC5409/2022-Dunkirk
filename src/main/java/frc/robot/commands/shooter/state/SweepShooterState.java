package frc.robot.commands.shooter.state;

import org.jetbrains.annotations.NotNull;

import frc.robot.Constants;
import frc.robot.base.Property;
import frc.robot.base.command.TimedStateCommand;
import frc.robot.base.shooter.SweepDirection;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.TargetType;
import frc.robot.subsystems.shooter.ShooterTurret;
import frc.robot.utils.Toggleable;

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
    protected final Property<SweepDirection> sweepDirection;
    protected final ShooterTurret turret;
    protected final Limelight     limelight;
    
    protected double  direction;
    protected boolean done;
    protected double  offset;
    

    public SweepShooterState(
        Limelight limelight,
        ShooterTurret turret, 
        Property<SweepDirection> sweepDirection
    ) {
        this.sweepDirection = sweepDirection;
        this.limelight = limelight;
        this.turret = turret;

        addRequirements(limelight, turret);
    }

    @Override
    public void initialize() {
        super.initialize();

        if (!limelight.isEnabled())
            limelight.enable();

        if (!turret.isEnabled())
            turret.enable();

        direction = (sweepDirection.get() == SweepDirection.kLeft) ? 1 : -1;
        offset = Constants.Shooter.SHOOTER_SWEEP_INVERSE.calculate(turret.getRotation());

        done = false;
    }

    @Override
    public void execute() {
        double time = getElapsedTime();

        if (limelight.getTargetType() == TargetType.kHub) {
            next("frc.robot.shooter:align");
            done = true;
        } else if (time / Constants.Shooter.SHOOTER_SWEEP_PERIOD > Constants.Shooter.SHOOTER_MAX_SWEEEP) {
            done = true;
        } else {
            turret.setRotationTarget(
                Constants.Shooter.SHOOTER_SWEEP_FUNCTION.calculate(offset + time*direction)
            );
        } 
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted || getNextState() == null) {
            limelight.disable();
            turret.disable();
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
