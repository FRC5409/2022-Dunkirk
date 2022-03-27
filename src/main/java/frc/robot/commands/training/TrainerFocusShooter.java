package frc.robot.commands.training;

import frc.robot.Constants;
import frc.robot.base.Property;
import frc.robot.base.ValueProperty;
import frc.robot.base.command.StateCommandGroup;
import frc.robot.base.shooter.SweepDirection;
import frc.robot.base.shooter.odometry.SimpleShooterOdometry;
import frc.robot.commands.shooter.state.AlignShooterState;
import frc.robot.commands.shooter.state.SearchShooterState;
import frc.robot.commands.shooter.state.SweepShooterState;
import frc.robot.commands.training.state.TrainerFocusShooterState;
import frc.robot.subsystems.Limelight;
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
public final class TrainerFocusShooter extends StateCommandGroup {
    private final Property<SimpleShooterOdometry> sharedOdometry;
    
    private final TrainerContext context;

    private final ShooterTurret turret;
    private final Limelight     limelight;

    public TrainerFocusShooter(
        TrainerDashboard dashboard,
        TrainerContext context,
        ShooterTurret turret,
        Limelight limelight,
        Property<SweepDirection> direction
    ) {
        sharedOdometry = new ValueProperty<>();

        addCommands(
            new SearchShooterState(limelight, false),
            new SweepShooterState(turret, limelight, sharedOdometry, direction),
            new AlignShooterState(turret, limelight, sharedOdometry),
            new TrainerFocusShooterState(dashboard, context, turret, limelight, sharedOdometry)
        );

        setDefaultState("frc.robot.shooter:search");
        
        this.turret = turret;
        this.limelight = limelight;
        this.context = context;
    }

    @Override
    public void initialize() {
        turret.enable();
        limelight.enable();
        
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

        turret.disable();
        limelight.disable();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}