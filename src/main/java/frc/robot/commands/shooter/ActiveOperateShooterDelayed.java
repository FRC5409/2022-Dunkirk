package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import frc.robot.utils.Equation;
import frc.robot.commands.shooter.state.ActiveDelayedAlignShooterState;
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
    private final PIDController sharedController;
    private final Equation turretOffsetModel;
    private final Equation flywheelOffsetModel;

    public ActiveOperateShooterDelayed(
        ShooterFlywheel flywheel,
        ShooterTurret turret,
        DriveTrain drivetrain,
        Limelight limelight,
        Indexer indexer,
        Property<SweepDirection> direction,
        Property<ShooterConfiguration> configuration,
        Property<Integer> offset,
        Property<Boolean> armed,
        Trigger trigger,
        Equation turretOffsetModel,
        Equation flywheelOffsetModel
    ) {
        sharedOdometry = new ValueProperty<>();
        sharedController = new PIDController(0.0,0.0,0.0);
        sharedController.setTolerance(0.3);
        
        this.turretOffsetModel = turretOffsetModel;
        this.flywheelOffsetModel = flywheelOffsetModel;

        addCommands(
            new SearchShooterState(limelight, true),
            new SweepShooterState(turret, limelight, Property.cast(sharedOdometry), direction),
            new ActiveDelayedAlignShooterState(turret, drivetrain, limelight, trigger, Property.cast(sharedOdometry), sharedController),
            new ActiveOperateArmShooterState(flywheel, turret, drivetrain, limelight, configuration, sharedOdometry, offset, armed, sharedController),
            new ActiveOperateRunShooterState(flywheel, turret, drivetrain, limelight, indexer, configuration, sharedOdometry, sharedController, offset)
        ); 

        setDefaultState("frc.robot.shooter:search");

        SmartDashboard.putNumber("Shooter P", SmartDashboard.getNumber("Shooter P", 0.0));
        SmartDashboard.putNumber("Shooter I", SmartDashboard.getNumber("Shooter I", 0.0));
        SmartDashboard.putNumber("Shooter D", SmartDashboard.getNumber("Shooter D", 0.0));
        SmartDashboard.putNumber("Shooter Output Thresh", SmartDashboard.getNumber("Shooter Output Thresh", 0.0));

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
                flywheelOffsetModel, 
                turretOffsetModel
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