package frc.robot.commands.shooter.state;

import org.jetbrains.annotations.NotNull;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.base.Property;
import frc.robot.base.StateCommandBase;
import frc.robot.base.shooter.ShooterConfiguration;
import frc.robot.base.shooter.ShooterModel;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.TargetType;
import frc.robot.subsystems.shooter.ShooterFlywheel;
import frc.robot.subsystems.shooter.ShooterTurret;

import frc.robot.utils.Toggleable;
import frc.robot.utils.Vector2;

import frc.robot.Constants;

// TODO update doc

/**
 * <h3>Operates the turret in the "Shooting" state.</h>
 * 
 * <p>In this state, the turret runs it's flywheel at a speed 
 * porportional to the distance of the turret from the outer port
 * and aligns the turret's rotation axis to the target.
 * Once both systems have reached their respective targets,
 * the indexer triggers, feeding powercells into the turret.</p>
 */
public class OperateDriveShooterState extends StateCommandBase {
    private final Property<ShooterConfiguration> configuration;
    private final ShooterFlywheel flywheel;
    private final ShooterTurret turret;
    private final DriveTrain drivetrain;
    private final Limelight limelight;
    private final Indexer indexer;
    
    private ShooterModel model;

    private double dt;
    private double dtexec;
    
    public OperateDriveShooterState(
        Limelight limelight,
        ShooterTurret turret,
        ShooterFlywheel flywheel,
        DriveTrain drivetrain,
        Indexer indexer,
        Property<ShooterConfiguration> configuration
    ) {
        this.configuration = configuration;
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.flywheel = flywheel;
        this.indexer = indexer;
        this.turret = turret;
        dt = 3;
        dtexec = 0;

        // we purposely do not require the drivetrain as to not
        // interrupt any currently executing drive commands
        addRequirements(limelight, turret, flywheel, indexer);
    }

    @Override
    public void initialize() {
        if (!Toggleable.isEnabled(limelight, turret, flywheel, indexer))
            throw new RuntimeException("Cannot operate shooter when requirements are not enabled.");

        //flywheel.spinFeeder(4500);
        model = configuration.get().getModel();
        dt = 3;

        Vector2 target = limelight.getTarget();
        double robotVelocity = drivetrain.getEncoderVelocity();
        double angleOfTurret = Math.toRadians(turret.getRotation());

        double distance = model.distance(target.y) + -1*robotVelocity* Math.cos(angleOfTurret)*dt;
        double velocity = model.calculate(distance);

        //speed lateral to the hub, or perpendicular to the direct distance from the hub.
        double lateralDist = robotVelocity * Math.sin(angleOfTurret)*dt;

        double turretNewPos = Math.tanh(lateralDist/distance);

        turret.setRotationTarget(turret.getRotation() + turretNewPos);

        // Set flywheel to estimated velocity
        flywheel.setVelocity(velocity);
        flywheel.spinFeeder(5000);
        dtexec = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        model = configuration.get().getModel();
        dt = 3;

        Vector2 target = limelight.getTarget();
        double robotVelocity = drivetrain.getEncoderVelocity();
        double angleOfTurret = Math.toRadians(turret.getRotation());

        double distance = model.distance(target.y) + -1*robotVelocity* Math.cos(angleOfTurret)*dt;
        double velocity = model.calculate(distance);

        //speed lateral to the hub, or perpendicular to the direct distance from the hub.
        double lateralDist = robotVelocity * Math.sin(angleOfTurret)*dt;

        double turretNewPos = Math.tanh(lateralDist/distance);

        turret.setRotationTarget(turret.getRotation() + turretNewPos);

        // Set flywheel to estimated velocity
        flywheel.setVelocity(velocity);
        flywheel.spinFeeder(5000);
        dtexec = System.currentTimeMillis();


        // Continue aligning shooter
        if (Math.abs(target.x) > (Constants.Vision.ALIGNMENT_THRESHOLD))
            turret.setRotationTarget(turret.getRotation() + (target.x) * Constants.Vision.ROTATION_P);

        if (turret.isTargetReached() && flywheel.isTargetReached()) {
            flywheel.spinFeeder(Constants.Shooter.FEEDER_VELOCITY);
            indexer.indexerOn(1);
        }

        drivetrain.displayEncoder();

    }

    @Override
    public void end(boolean interrupted) {
        flywheel.setVelocity(0);
        indexer.stopIndexer();
    }

    @Override
    public boolean isFinished() {
        return !(limelight.hasTarget() && limelight.getTargetType() == TargetType.kHub);
    }

    @Override
    public @NotNull String getStateName() {
        return "frc.robot.shooter:operate";
    }
}
