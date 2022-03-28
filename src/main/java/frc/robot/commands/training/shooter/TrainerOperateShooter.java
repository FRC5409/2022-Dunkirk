package frc.robot.commands.training.shooter;

import frc.robot.Constants;
import frc.robot.base.Property;
import frc.robot.base.ValueProperty;
import frc.robot.base.command.ProxyStateCommandGroup;
import frc.robot.base.shooter.SweepDirection;
import frc.robot.base.shooter.odometry.SimpleShooterOdometry;
import frc.robot.commands.shooter.state.AlignShooterState;
import frc.robot.commands.shooter.state.SearchShooterState;
import frc.robot.commands.shooter.state.SweepShooterState;
import frc.robot.commands.training.shooter.state.TrainerOperateShooterState;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.shooter.ShooterFlywheel;
import frc.robot.subsystems.shooter.ShooterTurret;
import frc.robot.training.TrainerContext;
import frc.robot.training.TrainerDashboard;

/**
 * This command runs the turret flywheel at a speed
 * porportional to the distance of the turret from the
 * shooting target and operates the indexer once the rpm
 * reaches it's setpoint.
 * 
 * @author Keith Davies
 */
public final class TrainerOperateShooter extends ProxyStateCommandGroup {
    private final Property<SimpleShooterOdometry> sharedOdometry;

    private final TrainerContext context;

    public TrainerOperateShooter(
        TrainerDashboard dashboard,
        TrainerContext context,
        ShooterFlywheel flywheel,
        ShooterTurret turret,
        Limelight limelight,
        Indexer indexer,
        Property<SweepDirection> direction,
        Property<Boolean> armed
    ) {
        sharedOdometry = new ValueProperty<>();
        
        addCommands(
            new SearchShooterState(limelight, false),
            new SweepShooterState(turret, limelight, sharedOdometry, direction),
            new AlignShooterState(turret, limelight, sharedOdometry),
            new TrainerOperateShooterState(dashboard, context, flywheel, turret, limelight, indexer, sharedOdometry, armed)
        );

        setDefaultState("frc.robot.shooter:search");
        
        this.context = context;
    }

    @Override
    public void initialize() {
        sharedOdometry.set(
            new SimpleShooterOdometry(
                context.getOdometryModel(), 
                Constants.Training.DEFAULT_TARGET_FILTER.create()     
            )
        );

        super.initialize();
    }


    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}