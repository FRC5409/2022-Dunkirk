package frc.robot.commands.training.shooter;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.base.Property;
import frc.robot.base.ValueProperty;
import frc.robot.base.command.ProxyStateGroupCommand;
import frc.robot.base.indexer.IndexerArmedState;
import frc.robot.base.shooter.ShooterConditions;
import frc.robot.base.shooter.ShooterConfiguration;
import frc.robot.base.shooter.ShooterState;
import frc.robot.base.shooter.SweepDirection;
import frc.robot.base.shooter.odometry.DriveShooterOdometry;
import frc.robot.base.training.TrainerContext;
import frc.robot.base.training.TrainerDashboard;
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
public final class TrainerOperateShooter extends ProxyStateGroupCommand {
    private final Property<ShooterConfiguration> shooterConfiguration;
    private final Property<DriveShooterOdometry> sharedOdometry;
    private final Property<ShooterState> shooterState;
    private final Property<Boolean> shooterTriggerDebounce;
    private final Property<Double> driveSpeed;

    private final TrainerDashboard dashboard;
    private final TrainerContext context;

    private final ShooterConditions shooterConditions;

    public TrainerOperateShooter(
        TrainerContext context,
        TrainerDashboard dashboard,
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
        shooterTriggerDebounce = new ValueProperty<>(false);
        shooterConditions = new ShooterConditions();
        sharedOdometry = new ValueProperty<>();

        addStates(
            new SearchShooterState(limelight, shooterState),
            new SweepShooterState(turret, limelight, Property.cast(sharedOdometry), shooterSweepDirection, shooterState),
            
            new OperateShooterState(turret, drivetrain, limelight, shooterConditions, 
                shooterConfiguration, sharedOdometry, indexerArmedState)
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

        this.context = context;
        this.dashboard = dashboard;
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
    public void execute() {
        super.execute();
        
        DriveShooterOdometry odometry = sharedOdometry.get();
        if (odometry.hasTarget()) {
            context.getConfiguration()
                .setDistance(odometry.getDistance());
        }

        dashboard.update();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        
        shooterConditions.reset();
        
        // Reconfigure properties
        driveSpeed.set(1.0);
        shooterState.set(ShooterState.kOff);
        sharedOdometry.set(null);
        shooterTriggerDebounce.set(false);
    }
}