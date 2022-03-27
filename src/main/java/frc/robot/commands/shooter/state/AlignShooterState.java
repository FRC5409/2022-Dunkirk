package frc.robot.commands.shooter.state;

import org.jetbrains.annotations.NotNull;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import frc.robot.base.Property;
import frc.robot.base.command.TimedStateCommand;
import frc.robot.base.shooter.odometry.SimpleShooterOdometry;

import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.TargetType;
import frc.robot.subsystems.shooter.ShooterTurret;

import frc.robot.utils.Toggleable;
import frc.robot.utils.Vector2;

// TODO update doc
public class AlignShooterState extends TimedStateCommand {
    protected final Property<SimpleShooterOdometry> sharedOdometry;

    protected final ShooterTurret turret;
    protected final Limelight limelight;
    
    protected SimpleShooterOdometry odometry;
    protected boolean done;

    public AlignShooterState(
        ShooterTurret turret,
        Limelight limelight,
        Property<SimpleShooterOdometry> sharedOdometry
    ) {
        this.sharedOdometry = sharedOdometry;
        this.limelight = limelight;
        this.turret = turret;

        addRequirements(limelight, turret);
    }

    @Override
    public void initialize() {
        super.initialize();

        if (!Toggleable.isEnabled(limelight))
            throw new RuntimeException("Cannot align shooter when requirements are not enabled.");

        if (!turret.isEnabled())
            turret.enable();

        odometry = sharedOdometry.get();

        done = false;
    }

    @Override
    public void execute() {
        if (limelight.getTargetType() == TargetType.kHub)
            odometry.update(limelight.getTargetPosition());

        if (odometry.hasTarget() && !odometry.isLost()) {
            Vector2 target = odometry.getTarget();

            if (Math.abs(target.x) < Constants.Vision.ALIGNMENT_THRESHOLD) {
                next("frc.robot.shooter:operate");
                done = true;
            } else if (getElapsedTime() > Constants.Shooter.ALIGNMENT_MAX_TIME) {
                done = true;
            } else {
                turret.setRotationTarget(turret.getRotation() + target.x * Constants.Vision.ROTATION_P);
            }

            if(Constants.kConfig.DEBUG)
                SmartDashboard.putNumber("Alignment Offset", target.x);
        } else {
            next("frc.robot.shooter:sweep");
            done = true;
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
        return "frc.robot.shooter:align";
    }
}
