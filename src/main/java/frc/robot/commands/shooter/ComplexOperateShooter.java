package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.base.Property;
import frc.robot.base.ValueProperty;
import frc.robot.base.command.ProxyStateGroupCommand;
import frc.robot.base.drive.DriveOdometry;
import frc.robot.base.indexer.IndexerArmedState;
import frc.robot.base.shooter.ShooterConditions;
import frc.robot.base.shooter.ShooterConfiguration;
import frc.robot.base.shooter.ShooterModelProvider;
import frc.robot.base.shooter.ShooterState;
import frc.robot.base.shooter.SweepDirection;
import frc.robot.base.shooter.TrackingController;
import frc.robot.base.shooter.odometry.DriveShooterOdometry;
import frc.robot.commands.shooter.state.OperateShooterState;
import frc.robot.commands.shooter.state.SearchShooterState;
import frc.robot.commands.shooter.state.SweepShooterState;
import frc.robot.commands.shooter.state.operate.ArmShooterState;
import frc.robot.commands.shooter.state.operate.DormantShooterState;
import frc.robot.commands.shooter.state.operate.RunShooterState;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.shooter.ShooterFlywheel;
import frc.robot.subsystems.shooter.ShooterTurret;

/**
 * This is a StateCommandGroup commmand that handles all the states for the shooter. 
 * The states are the Search state, Sweep state, Align state, and the OperateShooterStaged state. 
 * The default state is the search state. 
 * 
 * @author Keith Davies
 */
public class ComplexOperateShooter extends ProxyStateGroupCommand {
    protected final Property<ShooterConfiguration> shooterConfiguration;
    protected final Property<DriveShooterOdometry> sharedOdometry;
    protected final Property<TrackingController> sharedController;
    protected final Property<ShooterState> shooterState;
    protected final Property<Boolean> shooterTriggerDebounce;
    protected final Property<Double> driveSpeed;

    protected final ShooterModelProvider shooterModelProvider;
    protected final ShooterConditions shooterConditions;
    
    public ComplexOperateShooter(
        ShooterFlywheel flywheel,
        ShooterTurret turret,
        DriveOdometry drivetrain,
        Limelight limelight,
        Indexer indexer,
        Trigger shooterTrigger,
        ShooterModelProvider shooterModelProvider,
        Property<ShooterConfiguration> shooterConfiguration,
        Property<IndexerArmedState> indexerArmedState,
        Property<SweepDirection> shooterSweepDirection,
        Property<ShooterState> shooterState,
        Property<Double> shooterOffset,
        Property<Double> driveSpeed
    ) {
        sharedOdometry = new ValueProperty<>();
        sharedController = new ValueProperty<>();
        shooterConditions = new ShooterConditions();
        shooterTriggerDebounce = new ValueProperty<>(false);
        
        OperateShooterState state = new OperateShooterState(turret, drivetrain, limelight, shooterConditions, 
            sharedController, sharedOdometry, indexerArmedState);

        addStates(
            new SearchShooterState(limelight, shooterState),
            new SweepShooterState(turret, limelight, Property.cast(sharedOdometry), shooterSweepDirection, shooterState),
            state.addStates(
                new DormantShooterState(shooterTrigger, sharedOdometry, 
                    shooterState, shooterTriggerDebounce, driveSpeed),
                
                new ArmShooterState(flywheel, shooterConditions, shooterTrigger, shooterModelProvider,
                    sharedOdometry, indexerArmedState, shooterState, shooterTriggerDebounce, 
                    shooterOffset, driveSpeed),

                new RunShooterState(flywheel, turret, indexer, shooterTrigger, shooterModelProvider, sharedOdometry,
                    indexerArmedState, shooterState, shooterTriggerDebounce, 
                    shooterOffset, driveSpeed)
            )
        ); 

        setDefaultState("frc.robot.shooter.search");

        this.driveSpeed = driveSpeed;
        this.shooterState = shooterState;
        this.shooterModelProvider = shooterModelProvider;
        this.shooterConfiguration = shooterConfiguration;
    }
    
    @Override
    public void initialize() {
        // Initialize odometry
        sharedOdometry.set(
            new DriveShooterOdometry(
                shooterModelProvider.getOdometryModel(), 
                shooterModelProvider.getTrackingModel()
            )
        );

        sharedController.set(
            new TrackingController(shooterModelProvider.getTrackingModel())
        );

        shooterState.set(ShooterState.kOff);

        super.initialize();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        
        shooterConditions.reset();
        
        // Reconfigure properties
        driveSpeed.set(1.0);
        shooterState.set(ShooterState.kOff);
        sharedOdometry.set(null);
        sharedController.set(null);
        shooterTriggerDebounce.set(false);
    }
}