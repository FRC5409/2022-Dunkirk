package frc.robot.commands.training;

import frc.robot.base.StateCommandGroup;
import frc.robot.commands.shooter.AlignShooterState;
import frc.robot.commands.shooter.SearchShooterState;
import frc.robot.commands.shooter.SweepShooterState;
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
public final class TrainerLookShooter extends StateCommandGroup {
    private final ShooterTurret   turret;
    private final Limelight       limelight;

    public TrainerLookShooter(
        Limelight limelight,
        ShooterTurret turret,
        TrainerDashboard dashboard,
        TrainerContext context
    ) {
        this.turret = turret;
        this.limelight = limelight;

        addCommands(
            new SearchShooterState(limelight),
            new SweepShooterState(limelight, turret),
            new AlignShooterState(limelight, turret),
            new TrainerLookShooterState(limelight, turret, dashboard, context)
        );

        setDefaultState("frc.robot.shooter:search");
    }

    @Override
    public void initialize() {
        turret.enable();
        limelight.enable();

        super.initialize();
    }


    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        turret.disable();
        limelight.disable();
    }
}