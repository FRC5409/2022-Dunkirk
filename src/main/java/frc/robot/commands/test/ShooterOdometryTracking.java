package frc.robot.commands.test;

import frc.robot.Constants;
import frc.robot.base.Property;
import frc.robot.base.ValueProperty;
import frc.robot.base.command.ProxyStateCommandGroup;
import frc.robot.base.shooter.ShooterConfiguration;
import frc.robot.base.shooter.SweepDirection;
import frc.robot.base.shooter.odometry.DriveShooterOdometry;
import frc.robot.commands.shooter.state.SearchShooterState;
import frc.robot.commands.shooter.state.SweepShooterState;
import frc.robot.commands.test.state.AlignTrackingState;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.shooter.ShooterFlywheel;
import frc.robot.subsystems.shooter.ShooterTurret;
import frc.robot.training.TrainingModel3;

public class ShooterOdometryTracking extends ProxyStateCommandGroup {
    private final Property<DriveShooterOdometry> sharedOdometry;
    private final Property<ShooterConfiguration> configuration;
    private TrainingModel3 turretModel;
    
    public ShooterOdometryTracking(
        ShooterFlywheel flywheel,
        ShooterTurret turret,
        DriveTrain drivetrain,
        Limelight limelight,
        Indexer indexer,
        Property<SweepDirection> direction,
        Property<ShooterConfiguration> configuration,
        Property<Integer> offset,
        TrainingModel3 turretModel
    ) {
        sharedOdometry = new ValueProperty<>();

        addCommands(
            new SearchShooterState(limelight, true),
            new SweepShooterState(turret, limelight, Property.cast(sharedOdometry), direction),
            new AlignTrackingState(turret, drivetrain, limelight, sharedOdometry, configuration)
        ); 

        setDefaultState("frc.robot.shooter:search");

        this.configuration = configuration;
        this.turretModel = turretModel;
    }

    @Override
    public void initialize() {
        ShooterConfiguration config = configuration.get();
        
        // Initialize odometry
        sharedOdometry.set(
            new DriveShooterOdometry(
                config.getOdometryModel(),
                config.getTargetFilter(),
                x -> 0,
                turretModel
            )
        );

        super.initialize();
    }
}
