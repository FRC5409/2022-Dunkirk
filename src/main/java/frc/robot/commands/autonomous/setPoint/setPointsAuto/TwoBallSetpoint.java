package frc.robot.commands.autonomous.setPoint.setPointsAuto;


import frc.robot.Constants;
import frc.robot.base.Property;
import frc.robot.base.ValueProperty;
import frc.robot.base.shooter.ShooterConfiguration;
import frc.robot.base.shooter.ShooterMode;
import frc.robot.base.shooter.SweepDirection;
// commands
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.setPoint.MoveToDistance;
import frc.robot.commands.indexer.IndexerIntakeActive;
import frc.robot.commands.shooter.RotateTurret;
import frc.robot.commands.shooter.legacy.OperateShooter;
// subsystems
import frc.robot.subsystems.shooter.ShooterFlywheel;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.shooter.ShooterTurret;

public class TwoBallSetpoint extends SequentialCommandGroup {

    private final DriveTrain driveTrain;
    private final Intake intake;
    private final Indexer indexer;
    private final ShooterFlywheel flywheel;
    private final ShooterTurret turret;
    private final Limelight limelight;
    

    public TwoBallSetpoint(
        DriveTrain driveTrain,
        Intake intake,
        Indexer indexer,
        Limelight limelight,
        ShooterTurret turret,
        ShooterFlywheel flywheel,
        Property<ShooterConfiguration> shooterConfiguration,
        Property<SweepDirection> shooterSweepDirection,
        Property<Integer> shooterOffset
    ) {
        
        this.driveTrain = driveTrain;
        this.intake     = intake;
        this.indexer    = indexer;
        this.flywheel   = flywheel;
        this.turret     = turret;
        this.limelight  = limelight;

        addCommands(
            new ParallelRaceGroup(
                new MoveToDistance(driveTrain, -10f),
                new IndexerIntakeActive(indexer, intake),

                new OperateShooter(limelight, turret, flywheel, indexer, shooterSweepDirection, shooterConfiguration, shooterOffset),
                new RotateTurret(turret, 0)
            )

        );


        addRequirements(driveTrain, indexer, intake);
        
    }
}