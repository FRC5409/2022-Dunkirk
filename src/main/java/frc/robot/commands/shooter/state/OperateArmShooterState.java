package frc.robot.commands.shooter.state;

import org.jetbrains.annotations.NotNull;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.base.Property;
import frc.robot.base.StateCommandBase;
import frc.robot.base.shooter.ShooterConfiguration;
import frc.robot.base.shooter.ShooterModel;

import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.TargetType;
import frc.robot.subsystems.shooter.ShooterFlywheel;
import frc.robot.subsystems.shooter.ShooterTurret;

import frc.robot.utils.Toggleable;
import frc.robot.utils.Vector2;

import frc.robot.Constants;

// TODO update doc

public class OperateArmShooterState extends StateCommandBase {
    private final Property<ShooterConfiguration> configuration;
    private final Property<Integer> offset;
    private final Property<Boolean> armed;
    private final ShooterFlywheel flywheel;
    private final ShooterTurret turret;
    private final Limelight limelight;
    
    private ShooterModel model;
    private boolean done;
    private Vector2 target;
    
    public OperateArmShooterState(
        Limelight limelight,
        ShooterTurret turret,
        ShooterFlywheel flywheel,
        Property<ShooterConfiguration> configuration,
        Property<Integer> offset,
        Property<Boolean> armed
    ) {
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

        target = limelight.getTargetPosition();
        model = configuration.get().getModel();

        done = false;
    }

    @Override
    public void execute() {
        if (model == null)
            return;

        if (limelight.getTargetType() == TargetType.kHub)
            target = limelight.getTargetPosition();
            
        double distance = model.distance(target.y);
        double velocity = model.calculate(distance);

        // Set flywheel to estimated veloctity
        flywheel.setVelocity(velocity + offset.get());

        // Continue aligning shooter
        if (Math.abs(target.x) > Constants.Vision.ALIGNMENT_THRESHOLD)
            turret.setRotationTarget(turret.getRotation() + target.x * Constants.Vision.ROTATION_P);

        if (turret.isTargetReached() && flywheel.isTargetReached() && flywheel.feederReachedTarget() && armed.get()) {
            next("frc.robot.shooter:operate:run");
            done = true;
        }

        if (Constants.kConfig.DEBUG) {
            SmartDashboard.putNumber("Velocity Prediction", velocity);
            SmartDashboard.putNumber("Active Velocity", flywheel.getVelocity());
            
            SmartDashboard.putNumber("Distance Prediction (ft)", distance);
            SmartDashboard.putNumber("Aligninment Offset", target.x);
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
