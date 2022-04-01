package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.base.NullDrive;
import frc.robot.base.Property;
import frc.robot.base.ValueProperty;
import frc.robot.base.command.ProxyStateCommandGroup;
import frc.robot.base.indexer.IndexerArmedState;
import frc.robot.base.shooter.ShooterConditions;
import frc.robot.base.shooter.ShooterConfiguration;
import frc.robot.base.shooter.ShooterState;
import frc.robot.base.shooter.SweepDirection;
import frc.robot.base.shooter.odometry.DriveShooterOdometry;
import frc.robot.commands.shooter.state.OperateShooterState;
import frc.robot.commands.shooter.state.SearchShooterState;
import frc.robot.commands.shooter.state.SweepShooterState;
import frc.robot.commands.shooter.state.operate.ArmShooterState;
import frc.robot.commands.shooter.state.operate.DormantShooterState;
import frc.robot.commands.shooter.state.operate.RunShooterState;
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
public final class SimpleOperateShooter extends ProxyStateCommandGroup {
    private final Property<ShooterConfiguration> shooterConfiguration;
    private final Property<DriveShooterOdometry> sharedOdometry;
    private final Property<ShooterState> shooterState;
    private final Property<Boolean> shooterTriggerDebounce;
    private final Property<Double> driveSpeed;

    private final ShooterConditions shooterConditions;
    private final PIDController turretController;

    public SimpleOperateShooter(
        ShooterFlywheel flywheel,
        ShooterTurret turret,
        Limelight limelight,
        Indexer indexer,
        Property<ShooterConfiguration> shooterConfiguration,
        Property<IndexerArmedState> indexerArmedState,
        Property<SweepDirection> shooterSweepDirection
    ) {    
        // Simulated properties
        Property<ShooterState> shooterState = new ValueProperty<>(ShooterState.kOff);
        Property<Double> shooterOffset = new ValueProperty<>(0.0);
        Property<Double> driveSpeed = new ValueProperty<>(1.0);

        Trigger shooterTrigger = new Trigger(() -> true);

        sharedOdometry = new ValueProperty<>();

        shooterConditions = new ShooterConditions();

        turretController = new PIDController(0,0,0);
            MotorUtils.setGains(turretController, Constants.Shooter.TURRET_MANUAL_GAINS);
        turretController.setTolerance(Constants.Shooter.TURRET_MANUAL_THRESHOLD);

        shooterTriggerDebounce = new ValueProperty<>(false);

        addStates(
            new SearchShooterState(limelight, shooterState),
            new SweepShooterState(turret, limelight, Property.cast(sharedOdometry), shooterSweepDirection, shooterState),

            new OperateShooterState(flywheel, turret, new NullDrive(), limelight, 
                indexer, shooterTrigger, turretController, shooterConditions, 
                shooterConfiguration, sharedOdometry, indexerArmedState, shooterState, shooterOffset, driveSpeed)
                .addStates(
                    new DormantShooterState(shooterTrigger, sharedOdometry, 
                        shooterState, shooterTriggerDebounce, driveSpeed),
                    
                    new ArmShooterState(flywheel, shooterConditions, shooterTrigger, shooterConfiguration, 
                        sharedOdometry, indexerArmedState, shooterState, shooterTriggerDebounce, 
                        shooterOffset, driveSpeed),

                    new RunShooterState(flywheel, indexer, shooterTrigger, shooterConfiguration, sharedOdometry,
                        indexerArmedState, shooterState, shooterTriggerDebounce, 
                        shooterOffset, driveSpeed)
                )
               
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