package frc.robot.commands.shooter;

import frc.robot.Constants;
import frc.robot.base.StateCommandGroup;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.shooter.ShooterFlywheel;
import frc.robot.subsystems.shooter.ShooterTurret;

import frc.robot.utils.*;

/**
 * This command runs the turret flywheel at a speed
 * porportional to the distance of the turret from the
 * shooting target and operates the indexer once the rpm
 * reaches it's setpoint.
 * 
 * @author Keith Davies
 */
public final class OperateShooter extends StateCommandGroup {
    private final ShooterFlywheel flywheel;
    private final ShooterTurret   turret;
    private final Limelight       limelight;
    private final Indexer    indexer;

    private final ShooterModel    model;

    public OperateShooter(
        Limelight limelight,
        ShooterTurret turret,
        ShooterFlywheel flywheel,
        Indexer indexer,
        ShooterModel model
    ) {
        this.flywheel = flywheel;
        this.turret = turret;
        this.limelight = limelight;
        this.indexer = indexer;
        this.model = model;

        addCommands(
            new SearchShooterState(limelight),
            new SweepShooterState(limelight, turret),
            new AlignShooterState(limelight, turret),
            new OperateShooterState(limelight, turret, flywheel, indexer, model)
        );

        setDefaultState("frc.robot.shooter:search");
    }

    @Override
    public void initialize() {
        flywheel.enable();
        limelight.enable();
        turret.enable();
        
        indexer.enable();
        
        flywheel.setVelocity(
            model.calculate(Constants.Shooter.PRE_SHOOTER_DISTANCE)
        );

        super.initialize();
    }


    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        flywheel.disable();
        turret.disable();
        limelight.disable();

        indexer.disable();
    }
}