package frc.robot.commands.shooter.state;

import org.jetbrains.annotations.NotNull;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.base.Property;
import frc.robot.base.StateCommandBase;
import frc.robot.base.shooter.ShooterConfiguration;
import frc.robot.base.shooter.ShooterModel;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Limelight;
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
    private double prevTime;
    private boolean spunFeeder;
    private Timer timer;
    
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

        spunFeeder = false;
        timer = new Timer();
        timer.start();
        // we purposely do not require the drivetrain as to not
        // interrupt any currently executing drive commands
        addRequirements(limelight, turret, flywheel, indexer);
    }

    @Override
    public void initialize() {
        model = configuration.get().getModel();
        prevTime = timer.get();
    }

    @Override
    public void execute() {
        if(model == null) return;
        if(!spunFeeder){
            spunFeeder = true;
            flywheel.spinFeeder(Constants.Shooter.FEEDER_VELOCITY);
        }
        
        dt = 2*(timer.get() - prevTime);
        prevTime = timer.get();

        Vector2 target = limelight.getTarget();

        //assumings keiths model calc is done with feet. 
        double robotVelocity = Units.metersToFeet(drivetrain.getEncoderVelocity());

        //angle of the turret if it was aligned correctly
        double angleOfTurret = turret.getRotation() + target.x;
        double distance = model.distance(target.y) - robotVelocity* Math.cos(Math.toRadians(Math.abs(angleOfTurret)))*dt;

        //speed lateral to the hub, or perpendicular to the direct distance from the hub.
        double lateralDist = robotVelocity * Math.sin(Math.toRadians(angleOfTurret))*dt;
        double velocityForFlywheel = model.calculate(Math.hypot(distance, lateralDist));
        double turretNewPos = Math.toDegrees(Math.atan(lateralDist/distance));

        flywheel.setVelocity(velocityForFlywheel);
        turret.setRotationTarget(turret.getRotation() + (target.x+turretNewPos)*Constants.Vision.ROTATION_P);

        if (flywheel.feederReachedTarget()) {
            System.out.println("Indexer turning on");
            indexer.indexerOn(0.5);
        }

        System.out.println("DT: " + dt);
        System.out.println("Robot Velocity: " + robotVelocity + " ft/s");
        System.out.println("Angle of Turret: " + angleOfTurret);
        System.out.println("Distance: " + distance);
        System.out.println("Lateral distance prediction: " + lateralDist);
        System.out.println("Turret new pos: " + turretNewPos);
        System.out.println("velocity for flywheel: " + velocityForFlywheel);
        System.out.println("target.x: " + target.x);
        System.out.println("feeder target: " + flywheel.feederReachedTarget());
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
