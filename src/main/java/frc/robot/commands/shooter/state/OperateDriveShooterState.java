package frc.robot.commands.shooter.state;

import org.jetbrains.annotations.NotNull;

import edu.wpi.first.math.util.Units;
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

        // we purposely do not require the drivetrain as to not
        // interrupt any currently executing drive commands
        addRequirements(limelight, turret, flywheel, indexer);
    }

    @Override
    public void initialize() {
        if (!Toggleable.isEnabled(limelight, turret, flywheel, indexer))
            throw new RuntimeException("Cannot operate shooter when requirements are not enabled.");

        model = configuration.get().getModel();

        Vector2 target = limelight.getTarget();
        
        double robotVelocity = Units.metersToFeet(drivetrain.getEncoderVelocity());

        //angle between turret and velocity vector, assuming it is correctly aligned with the target.
        double angleOfTurret = Math.toRadians(turret.getRotation());

        double distance = model.distance(target.y) + -1*robotVelocity*Math.cos(angleOfTurret)*0.05;
        double velocityForFlywheel = model.calculate(distance);

        //speed lateral to the hub, or perpendicular to the direct distance from the hub.
        double lateralDist = robotVelocity * Math.sin(angleOfTurret)*0.05;

        double turretNewPos = Math.tanh(lateralDist/distance);

        turret.setRotationTarget(turret.getRotation() + (target.x-turretNewPos)*Constants.Vision.ROTATION_P);

        // Set flywheel to estimated velocity
        flywheel.setVelocity(velocityForFlywheel);
        flywheel.spinFeeder(5000);
        dt = System.currentTimeMillis() / 1000;
    }

    @Override
    public void execute() {
        dt = System.currentTimeMillis() / 1000 - dt;

        Vector2 target = limelight.getTarget();

        //assumings keiths model calc is done with feet. 
        double robotVelocity = Units.metersToFeet(drivetrain.getEncoderVelocity());

        //angle to the turret if it was aligned correctly
        double angleOfTurret = Math.toRadians(turret.getRotation()) + Math.toRadians(target.x);

        double distance = model.distance(target.y) + -1*robotVelocity* Math.cos(angleOfTurret)*dt;
        double velocityForFlywheel = model.calculate(distance);

        //speed lateral to the hub, or perpendicular to the direct distance from the hub.
        double lateralDist = robotVelocity * Math.sin(angleOfTurret)*dt;

        double turretNewPos = Math.tanh(lateralDist/distance);


        // Set flywheel to estimated velocity
        flywheel.setVelocity(velocityForFlywheel);


        // Continue aligning shooter
        if (Math.abs(target.x) > (Constants.Vision.ALIGNMENT_THRESHOLD + Math.abs(turretNewPos)))
            turret.setRotationTarget(turret.getRotation() + (target.x-turretNewPos)*Constants.Vision.ROTATION_P);

        if (turret.isTargetReached() && flywheel.isTargetReached() && flywheel.feederReachedTarget()) {
            indexer.indexerOn(1);
        }

        drivetrain.displayEncoder();

    }

    @Override
    public void end(boolean interrupted) {
        flywheel.setVelocity(0);
        flywheel.stopFeeder();
        indexer.stopIndexer();
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public @NotNull String getStateName() {
        return "frc.robot.shooter:operate";
    }
}
