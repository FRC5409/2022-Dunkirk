package frc.robot.commands.autonomous.setPoint.setPointsAuto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.base.Property;
import frc.robot.base.shooter.ShooterConfiguration;
import frc.robot.base.shooter.ShooterMode;
import frc.robot.base.shooter.SweepDirection;

import frc.robot.commands.indexer.IndexerIntakeActive;
import frc.robot.commands.indexer.RunIndexerBack;
import frc.robot.commands.SlowGear;
import frc.robot.commands.autonomous.setPoint.MoveToAngle;
import frc.robot.commands.autonomous.setPoint.MoveToDistance;
import frc.robot.commands.shooter.ConfigureShooter;
import frc.robot.commands.shooter.legacy.OperateShooter;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.shooter.ShooterFlywheel;
import frc.robot.subsystems.shooter.ShooterTurret;

public class FourBallSetpoint extends SequentialCommandGroup{

    DriveTrain m_drive;
    Intake m_intake;
    Indexer m_indexer;
    Limelight m_limelight;
    ShooterTurret m_turret;
    ShooterFlywheel m_flywheel;
    Property<ShooterConfiguration> m_shooterConfiguration;
    Property<SweepDirection> m_shooterSweepDirection;
    Property<Integer> m_shooterOffset;

    public FourBallSetpoint(
        DriveTrain drive,
        Intake intake,
        Indexer indexer,
        Limelight limelight,
        ShooterTurret turret,
        ShooterFlywheel shooterFlywheel,
        Property<ShooterConfiguration> shooterConfiguration,
        Property<SweepDirection> shooterSweepDirection,
        Property<Integer> shooterOffset
    ){

        m_drive   = drive;
        m_intake = intake;
        m_indexer = indexer;
        m_limelight = limelight;
        m_turret = turret;
        m_flywheel = shooterFlywheel;
        m_shooterConfiguration = shooterConfiguration;
        m_shooterSweepDirection = shooterSweepDirection;
        m_shooterOffset = shooterOffset;


        m_drive.setBrakeMode(true);

        /** TODO:
         *    Change distances
         *    Change angle
         *    Change Intake wait times
         */
        

        addCommands(
            new SlowGear(m_drive),

            // Get ball and shoot twice
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new IndexerIntakeActive(m_indexer, m_intake).withTimeout(0.5),
                    new ParallelRaceGroup(
                        new IndexerIntakeActive(m_indexer, m_intake),
                        new MoveToDistance(m_drive, 10f)
                    ),
                    new IndexerIntakeActive(m_indexer, m_intake).withTimeout(1),
                    new RunIndexerBack(m_intake, m_indexer).withTimeout(0.3)
                ),
                new ConfigureShooter(m_turret, m_limelight, m_shooterConfiguration, ShooterMode.kFar)
            ),
            new OperateShooter(m_limelight, m_turret, m_flywheel, m_indexer, m_shooterSweepDirection, m_shooterConfiguration, m_shooterOffset).withTimeout(3),
        
            // turn to next ball
            new MoveToAngle(m_drive, 90),

            // Move to next ball and intake
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new IndexerIntakeActive(m_indexer, m_intake).withTimeout(0.5),
                    new ParallelRaceGroup(
                        new IndexerIntakeActive(m_indexer, m_intake),
                        new MoveToDistance(m_drive, 10f)
                    ),
                    new IndexerIntakeActive(m_indexer, m_intake).withTimeout(1), // Tune this to wait for player
                    new RunIndexerBack(m_intake, m_indexer).withTimeout(0.3)
                ),
                new ConfigureShooter(m_turret, m_limelight, m_shooterConfiguration, ShooterMode.kFar)
            ),

            // Move to the hub and shoot
            new MoveToDistance(m_drive, -10),
            new OperateShooter(m_limelight, m_turret, m_flywheel, m_indexer, m_shooterSweepDirection, m_shooterConfiguration, m_shooterOffset).withTimeout(3)
        
        );
    }
}