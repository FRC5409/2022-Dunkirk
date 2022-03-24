package frc.robot.commands.training;

import frc.robot.base.Property;
import frc.robot.base.command.StateCommandGroup;
import frc.robot.base.shooter.ShooterTarget;
import frc.robot.base.shooter.SweepDirection;
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
    private final ShooterTurret turret;
    private final Limelight     limelight;
    private final ShooterTarget target;

    public TrainerFocusShooter(
        Limelight limelight,
        ShooterTurret turret,
        TrainerDashboard dashboard,
        TrainerContext context,
        Property<SweepDirection> direction
    ) {
        target = new ShooterTarget();

        addCommands(
            new SearchShooterState(limelight, false),
            new SweepShooterState(limelight, turret, target, direction),
            new AlignShooterState(limelight, turret, target),
            new TrainerFocusShooterState(limelight, turret, dashboard, context)
        );

        setDefaultState("frc.robot.shooter:search");
        
        this.turret = turret;
        this.limelight = limelight;
    }

    @Override
    public void initialize() {
        turret.enable();
        limelight.enable();
        
        target.reset();

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