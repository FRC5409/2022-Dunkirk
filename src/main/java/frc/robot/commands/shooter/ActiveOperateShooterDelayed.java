package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.base.Property;
import frc.robot.base.ValueProperty;
import frc.robot.base.command.ProxyStateCommandGroup;
import frc.robot.base.shooter.ShooterConfiguration;
import frc.robot.base.shooter.SweepDirection;
import frc.robot.base.shooter.odometry.DriveShooterOdometry;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.shooter.ShooterFlywheel;
import frc.robot.subsystems.shooter.ShooterTurret;

import frc.robot.commands.shooter.state.ActiveOperateArmShooterState;
import frc.robot.commands.shooter.state.ActiveOperateRunShooterState;
import frc.robot.commands.shooter.state.DelayedAlignShooterState;
import frc.robot.commands.shooter.state.SearchShooterState;
import frc.robot.commands.shooter.state.SweepShooterState;

/**
 * This is a StateCommandGroup commmand that handles all the states for the shooter. 
 * The states are the Search state, Sweep state, Align state, and the OperateShooterStaged state. 
 * The default state is the search state. 
 * 
 * @author Keith Davies
 */
public final class ActiveOperateShooterDelayed extends ProxyStateCommandGroup {
    private final Property<DriveShooterOdometry> sharedOdometry;
    private final Property<ShooterConfiguration> configuration;

    public ActiveOperateShooterDelayed(
        Limelight limelight,
        ShooterTurret turret,
        ShooterFlywheel flywheel,
        DriveTrain drivetrain,
        Indexer indexer,
        Property<SweepDirection> direction,
        Property<ShooterConfiguration> configuration,
        Property<Integer> offset,
        Property<Boolean> armed,
        Trigger trigger
    ) {
        sharedOdometry = new ValueProperty<>();

        addCommands(
            new SearchShooterState(limelight, true),
            new SweepShooterState(turret, limelight, Property.cast(sharedOdometry), direction),
            new DelayedAlignShooterState(turret, limelight, trigger, Property.cast(sharedOdometry)),
            new ActiveOperateArmShooterState(flywheel, turret, drivetrain, limelight, configuration, sharedOdometry, offset, armed),
            new ActiveOperateRunShooterState(flywheel, turret, drivetrain, limelight, indexer, configuration, sharedOdometry, offset)
        ); 

        setDefaultState("frc.robot.shooter:search");

        this.configuration = configuration;
    }
    
    @Override
    public void initialize() {
        ShooterConfiguration config = configuration.get();

        // Initialize odometry
        sharedOdometry.set(
            new DriveShooterOdometry(
                config.getOdometryModel(), 
                config.getTargetFilter(),
                Constants.Shooter.FLYWHEEL_OFFSET_MAPPING, 
                Constants.Shooter.TURRET_OFFSET_MAPPING
            )
        );

        super.initialize();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        
        // Destroy odometry
        sharedOdometry.set(null);
    }
}