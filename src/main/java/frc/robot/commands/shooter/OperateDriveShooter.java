package frc.robot.commands.shooter;

import frc.robot.base.Property;
import frc.robot.base.StateCommandGroup;
import frc.robot.base.shooter.ShooterConfiguration;
import frc.robot.base.shooter.SweepDirection;

import frc.robot.commands.shooter.state.AlignShooterState;
import frc.robot.commands.shooter.state.OperateDriveShooterState;
import frc.robot.commands.shooter.state.SearchShooterState;
import frc.robot.commands.shooter.state.SweepShooterState;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.shooter.ShooterFlywheel;
import frc.robot.subsystems.shooter.ShooterTurret;

import frc.robot.Constants;

/**
 * This is a StateCommandGroup commmand that handles all the states for the shooter. 
 * The states are the Search state, Sweep state, Align state, and the OperateShooter state. 
 * The default state is the search state. 
 * 
 * @author Keith Davies
 */
public final class OperateDriveShooter extends StateCommandGroup {
    private final Property<Integer> offset;
    private final ShooterFlywheel flywheel;
    private final ShooterTurret turret;
    private final Limelight limelight;
    private final Indexer indexer;

    public OperateDriveShooter(
        Limelight limelight,
        ShooterTurret turret,
        ShooterFlywheel flywheel,
        Indexer indexer,
        DriveTrain drivetrain, 
        Property<SweepDirection> direction,
        Property<ShooterConfiguration> configuration,
        Property<Integer> offset
    ) {

        addCommands(
            new SearchShooterState(limelight),
            new SweepShooterState(limelight, turret, direction),
            new AlignShooterState(limelight, turret),
            new OperateDriveShooterState(limelight, turret, flywheel, drivetrain, indexer, configuration)
        ); 

        setDefaultState("frc.robot.shooter:search");

        this.limelight = limelight;
        this.flywheel = flywheel;
        this.indexer = indexer;
        this.turret = turret;
        this.offset = offset;
    }

    @Override
    public void initialize() {
        limelight.enable();
        flywheel.enable();
        indexer.enable();
        turret.enable();

        flywheel.setVelocity(
            Constants.Shooter.PRE_SHOOTER_VELOCITY + offset.get()
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