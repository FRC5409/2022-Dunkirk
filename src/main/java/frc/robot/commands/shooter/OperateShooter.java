package frc.robot.commands.shooter;

import frc.robot.Constants;
import frc.robot.base.Property;
import frc.robot.base.StateCommandGroup;
import frc.robot.base.shooter.ShooterModel;
import frc.robot.base.shooter.SweepDirection;
import frc.robot.commands.shooter.state.AlignShooterState;
import frc.robot.commands.shooter.state.OperateShooterState;
import frc.robot.commands.shooter.state.SearchShooterState;
import frc.robot.commands.shooter.state.SweepShooterState;
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
    private final ShooterTurret turret;
    private final ShooterModel model;
    private final Limelight limelight;
    private final Indexer indexer;

    public OperateShooter(
        Limelight limelight,
        ShooterTurret turret,
        ShooterFlywheel flywheel,
        Indexer indexer,
        ShooterModel model,
        Property<SweepDirection> direction
    ) {
        addCommands(
            new SearchShooterState(limelight),
            new SweepShooterState(limelight, turret, direction),
            new AlignShooterState(limelight, turret),
            new OperateShooterState(limelight, turret, flywheel, indexer, model)
        );

        setDefaultState("frc.robot.shooter:search");

        this.limelight = limelight;
        this.flywheel = flywheel;
        this.indexer = indexer;
        this.turret = turret;
        this.model = model;
    }

    @Override
    public void initialize() {
        limelight.enable();
        flywheel.enable();
        indexer.enable();
        turret.enable();
        
        flywheel.setVelocity(
            model.calculate(Constants.Shooter.PRE_SHOOTER_DISTANCE)
        );

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