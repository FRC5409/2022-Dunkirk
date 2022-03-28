package frc.robot.commands.shooter.state;

import org.jetbrains.annotations.NotNull;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.base.Model4;
import frc.robot.base.Property;
import frc.robot.base.command.StateCommandBase;
import frc.robot.base.shooter.ShooterConfiguration;
import frc.robot.base.shooter.odometry.SimpleShooterOdometry;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.TargetType;
import frc.robot.subsystems.shooter.ShooterFlywheel;
import frc.robot.subsystems.shooter.ShooterTurret;

import frc.robot.utils.Toggleable;
import frc.robot.utils.Vector2;

import frc.robot.Constants;

// TODO update doc

public class OperateArmShooterState extends StateCommandBase {
    private final Property<SimpleShooterOdometry> sharedOdometry;
    private final Property<ShooterConfiguration> configuration;
    private final Property<Integer> offset;
    private final Property<Boolean> armed;

    private final ShooterFlywheel flywheel;
    private final ShooterTurret turret;
    private final Limelight limelight;
    
    private SimpleShooterOdometry odometry;
    private Model4 model;

    private boolean done;
    
    public OperateArmShooterState(
        ShooterFlywheel flywheel,
        ShooterTurret turret,
        Limelight limelight,
        Property<ShooterConfiguration> configuration,
        Property<SimpleShooterOdometry> sharedOdometry,
        Property<Integer> offset,
        Property<Boolean> armed
    ) {
        this.sharedOdometry = sharedOdometry;
        this.configuration = configuration;
        this.limelight = limelight;
        this.flywheel = flywheel;
        this.turret = turret;
        this.offset = offset;
        this.armed = armed;

        addRequirements(limelight, turret, flywheel);
    }

    @Override
    public void initialize() {
        if (!Toggleable.isEnabled(limelight, turret))
            throw new RuntimeException("Cannot arm shooter when requirements are not enabled.");

        if (!flywheel.isEnabled())
            flywheel.enable();
        
        // Obtain shooter configuration
        ShooterConfiguration config = configuration.get();

        model = config.getExecutionModel();
        odometry = sharedOdometry.get();

        done = false;
    }

    @Override
    public void execute() {
        if (model == null)
            return;

        // Update odometry target
        if (limelight.getTargetType() == TargetType.kHub)
            odometry.update(limelight.getTargetPosition());

        if (odometry.hasTarget()) {
            Vector2 target = odometry.getTarget();
                
            // Set flywheel to estimated veloctity
            double velocity = model.calculate(odometry.getDistance());
            flywheel.setVelocity(velocity + offset.get());

            // Continue aligning shooter
            if (Math.abs(target.x) > Constants.Vision.ALIGNMENT_THRESHOLD)
                turret.setRotationTarget(turret.getRotation() + target.x * Constants.Vision.ROTATION_P);

                
            if (Constants.kConfig.DEBUG) {
                SmartDashboard.putNumber("Velocity Prediction", velocity);
                SmartDashboard.putNumber("Aligninment Offset", target.x);
            }
        }

        if (turret.isTargetReached() && flywheel.isTargetReached() && armed.get()) {
            next("frc.robot.shooter:operate:run");
            done = true;
        }

        if (Constants.kConfig.DEBUG) {
            SmartDashboard.putNumber("Active Velocity", flywheel.getVelocity());
            
            SmartDashboard.putNumber("Distance Prediction (ft)", odometry.getDistance());
            SmartDashboard.putNumber("Velocity Offset", offset.get());
        }

    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted || getNextState() == null) {
            limelight.disable();
            flywheel.disable();
            turret.disable();
        }
    }

    @Override
    public boolean isFinished() {
        return done;
    }

    @Override
    public @NotNull String getStateName() {
        return "frc.robot.shooter:operate";
    }
}
