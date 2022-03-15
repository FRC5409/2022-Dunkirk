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
    private final Property<Integer> offset;
    private final ShooterFlywheel flywheel;
    private final ShooterTurret turret;
    private final DriveTrain drivetrain;
    private final Limelight limelight;
    private final Indexer indexer;
    
    private ShooterModel model;
    
    public OperateDriveShooterState(
        Limelight limelight,
        ShooterTurret turret,
        ShooterFlywheel flywheel,
        DriveTrain drivetrain,
        Indexer indexer,
        Property<ShooterConfiguration> configuration,
        Property<Integer> offset
    ) {
        this.configuration = configuration;
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.flywheel = flywheel;
        this.indexer = indexer;
        this.turret = turret;
        this.offset = offset;

        SmartDashboard.setDefaultNumber("Alignment Offset Slope", 0);
        SmartDashboard.putNumber("Alignment Offset Slope", SmartDashboard.getNumber("Alignment Offset Slope", 0));

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
    }

    @Override
    public void execute() {
        Vector2 target = limelight.getTarget();

        double distance = model.distance(target.y);
        double velocity = model.calculate(distance);

        double alignmentOffset = drivetrain.getEncoderVelocity() * SmartDashboard.getNumber("Alignment Offset Slope", 0);
        SmartDashboard.putNumber("Drive Rotation Offset", alignmentOffset);

        // Set flywheel to estimated veloctity
        flywheel.setVelocity(velocity + offset.get());

        // Continue aligning shooter
        if (Math.abs(target.x) > (Constants.Vision.ALIGNMENT_THRESHOLD + alignmentOffset))
            turret.setRotationTarget(turret.getRotation() + (target.x + alignmentOffset) * Constants.Vision.ROTATION_P);

        if (turret.isTargetReached() && flywheel.isTargetReached()) {
            flywheel.spinFeeder(Constants.Shooter.FEEDER_VELOCITY);
            indexer.indexerOn(1);
        }

        drivetrain.displayEncoder();

        if(Constants.kConfig.DEBUG){
            SmartDashboard.putNumber("Velocity Prediction", velocity);
            SmartDashboard.putNumber("Active Velocity", flywheel.getVelocity());
            
            SmartDashboard.putNumber("Distance Prediction (ft)", distance);
            SmartDashboard.putNumber("Aligninment Offset", target.x);
            SmartDashboard.putNumber("Velocity Offset", offset.get());
        }

    }

    @Override
    public void end(boolean interrupted) {
        flywheel.setVelocity(0);
        indexer.stopIndexer();
    }

    @Override
    public boolean isFinished() {
        return limelight.getTargetType() != TargetType.kHub;
    }

    @Override
    public @NotNull String getStateName() {
        return "frc.robot.shooter:operate";
    }
}
