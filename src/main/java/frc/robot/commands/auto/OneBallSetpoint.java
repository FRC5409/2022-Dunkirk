package frc.robot.commands.auto;


import frc.robot.Constants;
import frc.robot.base.ValueProperty;
import frc.robot.base.shooter.ShooterConfiguration;
import frc.robot.base.shooter.ShooterMode;
import frc.robot.base.shooter.ShooterModel;
import frc.robot.base.shooter.SweepDirection;

// commands
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.IndexerIntakeActive;
import frc.robot.commands.MoveToDistance;
import frc.robot.commands.shooter.OperateShooter;
import frc.robot.commands.shooter.RotateTurret;
// subsystems
import frc.robot.subsystems.shooter.ShooterFlywheel;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.shooter.ShooterTurret;

public class OneBallSetpoint extends SequentialCommandGroup {

    private final DriveTrain driveTrain;
    private final Intake intake;
    private final Indexer indexer;
    private final ShooterFlywheel flywheel;
    private final ShooterTurret turret;
    private final Limelight limelight;
    

    public OneBallSetpoint(DriveTrain driveTrain, Intake intake, Indexer indexer, Limelight limelight, ShooterTurret turret, ShooterFlywheel flywheel) {
        
        this.driveTrain = driveTrain;
        this.intake     = intake;
        this.indexer    = indexer;
        this.flywheel   = flywheel;
        this.turret     = turret;
        this.limelight  = limelight;

        ValueProperty<ShooterConfiguration> shooterConfiguration = new ValueProperty<ShooterConfiguration>(Constants.Shooter.CONFIGURATIONS.get(ShooterMode.kFar));
        ValueProperty<SweepDirection> shooterSweepDirection = new ValueProperty<SweepDirection>(SweepDirection.kLeft);

        addCommands(
            new OperateShooter(limelight, turret, flywheel, indexer, shooterSweepDirection, shooterConfiguration),
            new RotateTurret(turret, 0)
        );


        addRequirements(driveTrain, indexer, intake);
        
    }
}