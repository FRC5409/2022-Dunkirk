package frc.robot.commands.shooter.state;

import org.jetbrains.annotations.NotNull;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.base.TimedStateCommand;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.shooter.ShooterTurret;
import frc.robot.utils.Toggleable;
import frc.robot.utils.Vector2;

// TODO update doc
public class AlignShooterStagedState extends TimedStateCommand {
    private final ShooterTurret turret;
    private final Limelight limelight;
    private final Trigger trigger;
    
    private boolean done;

    public AlignShooterStagedState(Trigger trigger, Limelight limelight, ShooterTurret turret) {
        this.limelight = limelight;
        this.trigger = trigger;
        this.turret = turret;

        addRequirements(limelight, turret);
    }

    @Override
    public void initialize() {
        if (!Toggleable.isEnabled(limelight, turret))
            throw new RuntimeException("Cannot operate shooter when requirements are not enabled.");

        super.initialize();

        done = false;
    }

    @Override
    public void execute() {
        Vector2 target = limelight.getTarget();

        if (Math.abs(target.x) > Constants.Vision.ALIGNMENT_THRESHOLD)
            turret.setRotationTarget(turret.getRotation() + target.x);

        if (trigger.get()) {
            next("frc.robot.shooter:operate");
            done = true;
        }
        
        if(Constants.kConfig.DEBUG)
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
