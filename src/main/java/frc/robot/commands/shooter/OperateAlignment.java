package frc.robot.commands.shooter;

import frc.robot.Constants;
import frc.robot.base.Property;
import frc.robot.base.StateCommandGroup;
import frc.robot.base.shooter.ShooterConfiguration;
import frc.robot.base.shooter.SweepDirection;
import frc.robot.commands.shooter.state.AlignShooterState;
import frc.robot.commands.shooter.state.SearchShooterState;
import frc.robot.commands.shooter.state.SweepShooterState;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.shooter.ShooterFlywheel;
import frc.robot.subsystems.shooter.ShooterTurret;

public class OperateAlignment extends StateCommandGroup{

    private final Property<Integer> offset;
    private final ShooterTurret turret;
    private final Limelight limelight;

    public OperateAlignment(
        Limelight limelight,
        ShooterTurret turret,
        Property<SweepDirection> direction,
        Property<ShooterConfiguration> configuration,
        Property<Integer> offset
    ) {

        addCommands(
            new SearchShooterState(limelight),
            new SweepShooterState(limelight, turret, direction),
            new AlignShooterState(limelight, turret, true)
        ); 

        setDefaultState("frc.robot.shooter:search");

        this.limelight = limelight;
        this.turret = turret;
        this.offset = offset;
    }

    @Override
    public void initialize() {
        limelight.enable();
        turret.enable();

        super.initialize();
    }


    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        limelight.disable();
        turret.disable();
    }
    
}
