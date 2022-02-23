package frc.robot.commands.training;

import frc.robot.base.StateCommandGroup;
import frc.robot.commands.shooter.AlignShooterState;
import frc.robot.commands.shooter.SearchShooterState;
import frc.robot.commands.shooter.SweepShooterState;
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
public final class TrainerRunShooter extends StateCommandGroup {
    private final ShooterFlywheel flywheel;
    private final ShooterTurret   turret;
    private final Limelight       limelight;
    private final Indexer    indexer;

    public TrainerRunShooter(
        Limelight limelight,
        ShooterTurret turret,
        ShooterFlywheel flywheel,
        Indexer indexer,
        TrainerDashboard dashboard,
        TrainerContext context
    ) {
        this.flywheel = flywheel;
        this.turret = turret;
        this.limelight = limelight;
        this.indexer = indexer;

        addCommands(
            new SearchShooterState(limelight),
            new SweepShooterState(limelight, turret),
            new AlignShooterState(limelight, turret),
            new TrainerRunShooterState(limelight, turret, flywheel, indexer, dashboard, context)
        );

        setDefaultState("frc.robot.shooter:search");
    }

    @Override
    public void initialize() {
        flywheel.enable();
        turret.enable();
        limelight.enable();
        
        indexer.enable();

        super.initialize();
    }


    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        flywheel.disable();
        turret.disable();
        limelight.disable();

        //TODO check if you want indexer to be disabled.
        indexer.disable();
    }
}