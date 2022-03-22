package frc.robot.commands.autonomous.setPoint.setPointsAuto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.base.Property;
import frc.robot.base.shooter.ShooterConfiguration;
import frc.robot.base.shooter.ShooterMode;
import frc.robot.base.shooter.SweepDirection;

import frc.robot.commands.IndexerIntakeActive;
import frc.robot.commands.MoveToAngle;
import frc.robot.commands.MoveToDistance;
import frc.robot.commands.ResetOdometry;
import frc.robot.commands.RunIndexerBack;
import frc.robot.commands.SlowGear;
import frc.robot.commands.TurnToAngleGyro;
import frc.robot.commands.shooter.ConfigureShooter;
import frc.robot.commands.shooter.OperateShooter;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.shooter.ShooterFlywheel;
import frc.robot.subsystems.shooter.ShooterTurret;

public class MoveOnlyAuto extends SequentialCommandGroup{

    DriveTrain m_drive;
    Intake m_intake;
    Indexer m_indexer;
    Limelight m_limelight;
    ShooterTurret m_turret;
    ShooterFlywheel m_flywheel;
    Property<ShooterConfiguration> m_shooterConfiguration;
    Property<SweepDirection> m_shooterSweepDirection;
    Property<Integer> m_shooterOffset;

    public MoveOnlyAuto(
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

            // Move to ball 2
            new MoveToDistance(m_drive, 10f),

            // turn to human player
            new MoveToAngle(m_drive, 90),

            // move to human player
            new MoveToDistance(m_drive, 10f),

            // wait while intaking
            new IndexerIntakeActive(indexer, intake).withTimeout(2),

            // Move to the hub and shoot
            new MoveToDistance(m_drive, -10)
            //new OperateShooter(m_limelight, m_turret, m_flywheel, m_indexer, m_shooterSweepDirection, m_shooterConfiguration, m_shooterOffset).withTimeout(3)
        
        );
    }
}