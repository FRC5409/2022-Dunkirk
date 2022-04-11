package frc.robot.commands.shooter.legacy;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.base.ConstantProperty;
import frc.robot.base.Property;
import frc.robot.base.ValueProperty;
import frc.robot.base.drive.DriveOdometry;
import frc.robot.base.drive.MockDriveOdometry;
import frc.robot.base.indexer.IndexerArmedState;
import frc.robot.base.shooter.ConstantModelProvider;
import frc.robot.base.shooter.ShooterConfiguration;
import frc.robot.base.shooter.ShooterModelProvider;
import frc.robot.base.shooter.ShooterState;
import frc.robot.base.shooter.SweepDirection;
import frc.robot.commands.shooter.ComplexOperateShooter;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.shooter.ShooterFlywheel;
import frc.robot.subsystems.shooter.ShooterTurret;

/**
 * Legacy command for backwards compatibilty.
 * 
 * <b> Do not use in any new code. </b>
 * 
 * @deprecated Use {@link frc.robot.commands.shooter.ComplexOperateShooter ComplexOperateShooter} instead.
 */
@Deprecated
public class OperateShooter extends ParallelCommandGroup {
    private final Property<ShooterConfiguration> shooterConfiguration;
    private final Property<SweepDirection> shooterSweepDirection;
    private final Property<Integer> shooterOffset;
    
    private final ShooterFlywheel flywheel;
    private final ShooterTurret turret;
    private final Limelight limelight;
    private final Indexer indexer;

    public OperateShooter(
        Limelight limelight, 
        ShooterTurret turret,
        ShooterFlywheel flywheel,
        Indexer indexer,
        Property<SweepDirection> shooterSweepDirection,
        Property<ShooterConfiguration> shooterConfiguration,
        Property<Integer> shooterOffset
    ) {
        this.shooterSweepDirection = shooterSweepDirection;
        this.shooterConfiguration = shooterConfiguration;
        this.shooterOffset = shooterOffset;
        this.limelight = limelight;
        this.flywheel = flywheel;
        this.indexer = indexer;
        this.turret = turret;

        configureCommands();
    }

    private void configureCommands() {
        // Mock Drivetrain
        DriveOdometry mockDrivetrain = new MockDriveOdometry();
        
        // Mock Shooter Trigger
        Trigger mockShooterTrigger = new Trigger(() -> true);

        // Mock model provider
        ShooterModelProvider shooterModelProvider = new ConstantModelProvider(
            Constants.Shooter.ODOMETRY_MODEL,
            Constants.Shooter.TRACKING_MODEL,
            Constants.Shooter.EXECUTION_MODEL
        );
        
        // Mock Properties
        Property<IndexerArmedState> mockIndexerArmedState = new ConstantProperty<>(IndexerArmedState.kArmed);
        Property<ShooterState> mockShooterState = new ValueProperty<>(ShooterState.kOff);
        Property<Double> mockDriveSpeed = new ValueProperty<>(1.0);

        addCommands(
            new ComplexOperateShooter(
                flywheel, turret, mockDrivetrain, limelight, indexer, 
                mockShooterTrigger, shooterModelProvider, shooterConfiguration, 
                mockIndexerArmedState, shooterSweepDirection, mockShooterState, Property.cast(shooterOffset), mockDriveSpeed)
        );
    }
}
