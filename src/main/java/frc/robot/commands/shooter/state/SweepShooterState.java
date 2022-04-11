package frc.robot.commands.shooter.state;

import org.jetbrains.annotations.NotNull;

import frc.robot.Constants;
import frc.robot.base.Property;
import frc.robot.base.command.InterruptType;
import frc.robot.base.command.TimedStateCommand;
import frc.robot.base.shooter.ShooterState;
import frc.robot.base.shooter.SweepDirection;
import frc.robot.base.shooter.odometry.SimpleShooterOdometry;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.TargetType;
import frc.robot.subsystems.shooter.ShooterTurret;
import frc.robot.subsystems.shooter.ShooterTurret.ReferenceType;

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
    protected final Property<SimpleShooterOdometry> sharedOdometry;
    protected final Property<SweepDirection> sweepDirection;
    protected final Property<ShooterState> shooterState;

    protected final ShooterTurret turret;
    protected final Limelight limelight;
    
    protected SimpleShooterOdometry odometry;
    protected double direction;
    protected double offset;
    private boolean done;

    public SweepShooterState(
        ShooterTurret turret, 
        Limelight limelight,
        Property<SimpleShooterOdometry> sharedOdometry,
        Property<SweepDirection> sweepDirection,
        Property<ShooterState> shooterState
    ) {
        this.sharedOdometry = sharedOdometry;
        this.sweepDirection = sweepDirection;
        this.shooterState = shooterState;
        this.limelight = limelight;
        this.turret = turret;

        addRequirements(limelight, turret);
    }

    @Override
    public void initialize() {
        super.initialize();

        if (!limelight.isEnabled()) {
            getLogger().severe("Limelight was unexpectedly disabled, enabling...");
            limelight.enable();
        }

        if (!turret.isEnabled()) {
            getLogger().severe("Turret was unexpectedly disabled, enabling...");
            turret.enable();
        }

        odometry = sharedOdometry.get();

        if (odometry.getTarget() != null) {
            direction = (odometry.getTarget().x > 0) ? -1 : 1;
            odometry.reset();

            System.out.println("Using target to predict sweep");
        } else {
            direction = (sweepDirection.get() == SweepDirection.kLeft) ? 1 : -1;

            System.out.println("Using swwep direction to predict sweep");
        }

        offset = Constants.Shooter.SHOOTER_SWEEP_INVERSE.calculate(turret.getRotation());
        done = false;

        shooterState.set(ShooterState.kSweep);
    }

    @Override
    public void execute() {
        double time = getElapsedTime();

        if (limelight.getTargetType() == TargetType.kHub) {
            odometry.update(limelight.getTargetPosition());
            next("frc.robot.shooter.operate:dormant");
        //} else if (time / Constants.Shooter.SHOOTER_SWEEP_PERIOD > Constants.Shooter.SHOOTER_MAX_SWEEEP) {
        //    done = true;
        } else {
            turret.setReference(
                Constants.Shooter.SHOOTER_SWEEP_FUNCTION.calculate(offset + time*direction),
                ReferenceType.kRotation
            );
        } 
    }

    @Override
    public void end(InterruptType interrupt) {
        if (interrupt == InterruptType.kCancel || getNextState() == null) {
            limelight.disable();
            turret.disable();
        }
    }

    @Override
    public boolean isFinished() {
        return done;
    }

    @Override
    public @NotNull String getName() {
        return "frc.robot.shooter.sweep";
    }
}
