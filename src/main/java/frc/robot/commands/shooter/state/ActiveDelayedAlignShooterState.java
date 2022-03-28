package frc.robot.commands.shooter.state;

import org.jetbrains.annotations.NotNull;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.base.Property;
import frc.robot.base.command.TimedStateCommand;
import frc.robot.base.shooter.odometry.DriveShooterOdometry;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.TargetType;
import frc.robot.subsystems.shooter.ShooterTurret;
import frc.robot.utils.Toggleable;
import frc.robot.utils.Vector2;

// TODO update doc
public class ActiveDelayedAlignShooterState extends TimedStateCommand {
    private Property<DriveShooterOdometry> sharedOdometry;
    
    private final ShooterTurret turret;
    private final DriveTrain drivetrain;
    private final Limelight limelight;
    private final Trigger trigger;
    
    private DriveShooterOdometry odometry;
    private boolean done;

    public ActiveDelayedAlignShooterState(
        ShooterTurret turret,
        DriveTrain drivetrain,
        Limelight limelight,
        Trigger trigger,
        Property<DriveShooterOdometry> sharedOdometry
    ) {
        this.sharedOdometry = sharedOdometry;
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.trigger = trigger;
        this.turret = turret;
        
        addRequirements(limelight, turret);
    }

    @Override
    public void initialize() {
        super.initialize();

        if (!Toggleable.isEnabled(limelight))
            throw new RuntimeException("Cannot align shooter when requirements are not enabled.");
        else if (sharedOdometry.get() == null)
            throw new RuntimeException("Cannot operate shooter when odometry is not initialized");

        if (!turret.isEnabled())
            turret.enable();

        odometry = sharedOdometry.get();

        done = false;
    }

    @Override
    public void execute() {
        Vector2 target = limelight.getTargetPosition();

        // Update odometry target
        if (limelight.getTargetType() == TargetType.kHub) {
            odometry.update(
                limelight.getTargetPosition(),
                drivetrain.getEncoderVelocity(),
                turret.getRotation()
            );
        }
            
        // Continue aligning shooter
        turret.setRotationTarget(
            MathUtil.interpolate(
                turret.getRotationTarget(),
                odometry.getTurretOffset() - odometry.getRotation(),
                SmartDashboard.getNumber("Rotation Smoothing", 0)
            )
        );

        if (trigger.get()) {
            next("frc.robot.shooter:operate");
            done = true;
        }
        
        if(Constants.kConfig.DEBUG)
            SmartDashboard.putData("Robot Odometry", odometry);
    }

    @Override
    public boolean isFinished() {
        return (limelight.getTargetType() != TargetType.kHub) || done;
    }

    @Override
    public @NotNull String getStateName() {
        return "frc.robot.shooter:align";
    }
}
