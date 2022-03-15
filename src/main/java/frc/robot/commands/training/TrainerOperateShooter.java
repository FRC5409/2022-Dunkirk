package frc.robot.commands.training;

import frc.robot.base.Property;
import frc.robot.base.StateCommandGroup;
import frc.robot.base.shooter.SweepDirection;
import frc.robot.commands.shooter.state.AlignShooterState;
import frc.robot.commands.shooter.state.SearchShooterState;
import frc.robot.commands.shooter.state.SweepShooterState;
import frc.robot.commands.training.state.TrainerOperateShooterState;
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
public final class TrainerOperateShooter extends StateCommandGroup {
    private final ShooterFlywheel flywheel;
    private final ShooterTurret   turret;
    private final Limelight       limelight;
    private final Indexer         indexer;

    public TrainerOperateShooter(
        Limelight limelight,
        ShooterTurret turret,
        ShooterFlywheel flywheel,
        Indexer indexer,
        TrainerDashboard dashboard,
        TrainerContext context,
        Property<SweepDirection> direction
    ) {
        addCommands(
            new SearchShooterState(limelight, false),
            new SweepShooterState(limelight, turret, direction),
            new AlignShooterState(limelight, turret),
            new TrainerOperateShooterState(limelight, turret, flywheel, indexer, dashboard, context)
        );

        setDefaultState("frc.robot.shooter:search");
        
        this.flywheel = flywheel;
        this.turret = turret;
        this.limelight = limelight;
        this.indexer = indexer;
    }

    @Override
    public void initialize() {
        limelight.enable();
        flywheel.enable();
        indexer.enable();
        turret.enable();

        super.initialize();
    }


    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        limelight.disable();
        flywheel.disable();
        indexer.disable();
        turret.disable();
    }
}