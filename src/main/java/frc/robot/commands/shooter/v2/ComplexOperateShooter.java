package frc.robot.commands.shooter.v2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.base.Property;
import frc.robot.base.ValueProperty;
import frc.robot.base.command.ProxyStateCommandGroup;
import frc.robot.base.indexer.IndexerArmedState;
import frc.robot.base.shooter.ShooterConditions;
import frc.robot.base.shooter.ShooterConfiguration;
import frc.robot.base.shooter.ShooterState;
import frc.robot.base.shooter.SweepDirection;
import frc.robot.base.shooter.odometry.DriveShooterOdometry;
import frc.robot.commands.shooter.v2.state.OperateShooterState;
import frc.robot.commands.shooter.v2.state.SearchShooterState;
import frc.robot.commands.shooter.v2.state.SweepShooterState;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.shooter.ShooterFlywheel;
import frc.robot.subsystems.shooter.ShooterTurret;
import frc.robot.utils.MotorUtils;

/**
 * This is a StateCommandGroup commmand that handles all the states for the shooter. 
 * The states are the Search state, Sweep state, Align state, and the OperateShooterStaged state. 
 * The default state is the search state. 
 * 
 * @author Keith Davies
 */
public final class ComplexOperateShooter extends ProxyStateCommandGroup {
    private final Property<ShooterConfiguration> shooterConfiguration;
    private final Property<DriveShooterOdometry> sharedOdometry;
    private final Property<ShooterState> shooterState;
    private final Property<Boolean> shooterTriggerDebounce;
    private final Property<Double> driveSpeed;

    private final ShooterConditions shooterConditions;
    private final PIDController turretController;

    public ComplexOperateShooter(
        ShooterFlywheel flywheel,
        ShooterTurret turret,
        DriveTrain drivetrain,
        Limelight limelight,
        Indexer indexer,
        Trigger shooterTrigger,
        Property<ShooterConfiguration> shooterConfiguration,
        Property<IndexerArmedState> indexerArmedState,
        Property<SweepDirection> shooterSweepDirection,
        Property<ShooterState> shooterState,
        Property<Double> shooterOffset,
        Property<Double> driveSpeed
    ) {
        sharedOdometry = new ValueProperty<>();

        shooterConditions = new ShooterConditions();

        turretController = new PIDController(0,0,0);
            MotorUtils.setGains(turretController, Constants.Shooter.TURRET_MANUAL_GAINS);
        turretController.setTolerance(Constants.Shooter.TURRET_MANUAL_THRESHOLD);

        shooterTriggerDebounce = new ValueProperty<>(false);

        addStates(
            new SearchShooterState(limelight, shooterState),
            new SweepShooterState(turret, limelight, Property.cast(sharedOdometry), shooterSweepDirection, shooterState),
            
            new OperateShooterState(flywheel, turret, drivetrain, limelight, 
                indexer, shooterTrigger, turretController, shooterConditions, 
                shooterConfiguration, sharedOdometry, indexerArmedState, 
                shooterState, shooterOffset, driveSpeed)
        ); 

        setDefaultState("frc.robot.shooter.search");

        this.driveSpeed = driveSpeed;
        this.shooterState = shooterState;
        this.shooterConfiguration = shooterConfiguration;
    }
    
    @Override
    public void initialize() {
        ShooterConfiguration config = shooterConfiguration.get();

        // Initialize odometry
        sharedOdometry.set(
            new DriveShooterOdometry(
                config.getOdometryModel(), 
                config.getTrackingModel()
            )
        );

        shooterState.set(ShooterState.kOff);

        super.initialize();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        
        shooterConditions.reset();
        turretController.reset();
        
        // Reconfigure properties
        driveSpeed.set(1.0);
        shooterState.set(ShooterState.kOff);
        sharedOdometry.set(null);
        shooterTriggerDebounce.set(false);
    }
}