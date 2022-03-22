package frc.robot.commands.shooter.state;

import org.jetbrains.annotations.NotNull;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.base.TimedStateCommand;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.TargetType;
import frc.robot.subsystems.shooter.ShooterTurret;
import frc.robot.utils.Toggleable;
import frc.robot.utils.Vector2;

// TODO update doc
public class AlignShooterState extends TimedStateCommand {
    private final ShooterTurret turret;
    private final Limelight limelight;
    private boolean done;
    private boolean auto;


    public AlignShooterState(Limelight limelight, ShooterTurret turret) {
        this.limelight = limelight;
        this.turret = turret;

        done = false;
        auto = true;
        addRequirements(limelight, turret);
    }

    /**
     * This constructor contains a flag. Placing false or true in the place of flag causes the alignment to
     * be done as if the driver is shooting as oppose to auto. 
     * @param limelight
     * @param turret
     * @param flag
     */
    public AlignShooterState(Limelight limelight, ShooterTurret turret, boolean flag) {
        this.limelight = limelight;
        this.turret = turret;

        done = false;
        auto = false;
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

        if(auto){
            if (Math.abs(target.x) < Constants.Vision.ALIGNMENT_THRESHOLD) {
                next("frc.robot.shooter:operate");
                done = true;
            } else if (getElapsedTime() > Constants.Shooter.ALIGNMENT_MAX_TIME)
                done = true;
            else
                turret.setRotationTarget(turret.getRotation() + target.x*Constants.Vision.ROTATION_P);
        } else {
            if (!(limelight.hasTarget() && limelight.getTargetType() == TargetType.kHub)){
                next("frc.robot.shooter:sweep");
            }
            if (!(Math.abs(target.x) < Constants.Vision.ALIGNMENT_THRESHOLD)) 
                turret.setRotationTarget(turret.getRotation() + target.x*Constants.Vision.ROTATION_P);
                
            if(Constants.kConfig.DEBUG) SmartDashboard.putNumber("Alignment Offset", target.x);
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
