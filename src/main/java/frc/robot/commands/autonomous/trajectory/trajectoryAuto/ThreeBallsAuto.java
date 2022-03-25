package frc.robot.commands.autonomous.trajectory.trajectoryAuto;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.ProxyScheduleCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Shooter;
import frc.robot.Constants.kAuto;
import frc.robot.base.Property;
import frc.robot.base.shooter.ShooterConfiguration;
import frc.robot.base.shooter.ShooterMode;
import frc.robot.base.shooter.SweepDirection;
import frc.robot.commands.ConfigureProperty;
import frc.robot.commands.SlowGear;
import frc.robot.commands.autonomous.trajectory.ResetOdometry;
import frc.robot.commands.indexer.IndexerIntakeActive;
import frc.robot.commands.indexer.RunIndexerBack;
import frc.robot.commands.shooter.ConfigureShooter;
import frc.robot.commands.shooter.OperateShooter;
import frc.robot.commands.shooter.RotateTurret;
import frc.robot.commands.shooter.state.SweepShooterState;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.shooter.ShooterFlywheel;
import frc.robot.subsystems.shooter.ShooterTurret;

public class ThreeBallsAuto extends SequentialCommandGroup{

    DriveTrain m_drive;
    Intake m_intake;
    Indexer m_indexer;
    Limelight m_limelight;
    ShooterTurret m_turret;
    ShooterFlywheel m_flywheel;
    Property<ShooterConfiguration> m_shooterConfiguration;
    Property<SweepDirection> m_shooterSweepDirection;
    Property<Integer> m_shooterOffset;

    public ThreeBallsAuto(
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
        m_intake  = intake;
        m_indexer = indexer;
        m_limelight = limelight;
        m_turret = turret;
        m_flywheel = shooterFlywheel;
        m_shooterConfiguration = shooterConfiguration;
        m_shooterSweepDirection = shooterSweepDirection;
        m_shooterOffset = shooterOffset;

                
        Trajectory t1 = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
                                                                   List.of(),
                                                                   new Pose2d(1.5/kAuto.kDistanceRatio, 0, new Rotation2d(0)),
                                                                   kAuto.configForwards);

        Trajectory t2 = TrajectoryGenerator.generateTrajectory(new Pose2d(1.5/kAuto.kDistanceRatio, 0, new Rotation2d(0)),
                                                                   List.of(),
                                                                   new Pose2d(5.45/kAuto.kDistanceRatio, -0.5/kAuto.kDistanceRatio, new Rotation2d(Math.PI*5/36)),
                                                                   kAuto.configForwards);

        Trajectory t3 = TrajectoryGenerator.generateTrajectory(new Pose2d(5.45/kAuto.kDistanceRatio, -0.5/kAuto.kDistanceRatio, new Rotation2d(Math.PI*5/36)),
                                                                   List.of(),
                                                                   new Pose2d(1.5/kAuto.kDistanceRatio, 0, new Rotation2d(0)),
                                                                   kAuto.configBackwards);

        RamseteCommand r1 = new RamseteCommand(t1, m_drive::getPose,
        new RamseteController(kAuto.kRamseteB, kAuto.kRamseteZeta),
        new SimpleMotorFeedforward(kAuto.ksVolts, 
                                   kAuto.kvVoltSecondsPerMeter,
                                   kAuto.kaVoltSecondsSquaredPerMeter),
        kAuto.kDriveKinematics, m_drive::getWheelSpeeds,
        new PIDController(kAuto.kPDriveVel, 0, 0), 
        new PIDController(kAuto.kPDriveVel, 0, 0),
        m_drive::tankDriveVolts, 
        m_drive); 

        RamseteCommand r2 = new RamseteCommand(t2, m_drive::getPose,
        new RamseteController(kAuto.kRamseteB, kAuto.kRamseteZeta),
        new SimpleMotorFeedforward(kAuto.ksVolts, 
                                   kAuto.kvVoltSecondsPerMeter,
                                   kAuto.kaVoltSecondsSquaredPerMeter),
        kAuto.kDriveKinematics, m_drive::getWheelSpeeds,
        new PIDController(kAuto.kPDriveVel, 0, 0), 
        new PIDController(kAuto.kPDriveVel, 0, 0),
        m_drive::tankDriveVolts, 
        m_drive); 


        RamseteCommand r3 = new RamseteCommand(t3, m_drive::getPose,
        new RamseteController(kAuto.kRamseteB, kAuto.kRamseteZeta),
        new SimpleMotorFeedforward(kAuto.ksVolts, 
                                   kAuto.kvVoltSecondsPerMeter,
                                   kAuto.kaVoltSecondsSquaredPerMeter),
        kAuto.kDriveKinematics, m_drive::getWheelSpeeds,
        new PIDController(kAuto.kPDriveVel, 0, 0), 
        new PIDController(kAuto.kPDriveVel, 0, 0),
        m_drive::tankDriveVolts, 
        m_drive); 

        m_drive.setBrakeMode(true);

        addCommands(

            // trajectory testing code
            // new SlowGear(m_drive),
            // new ResetOdometry(t1.getInitialPose(), m_drive),
            // r1, 
            // new ResetOdometry(t2.getInitialPose(), m_drive),
            // r2, 
            // new ResetOdometry(t3.getInitialPose(), m_drive),
            // r3

            // V1: run intake while getting to the human station
            new SlowGear(m_drive),
            new ConfigureShooter(m_turret, m_limelight, m_shooterConfiguration, ShooterMode.kFar),
            new ResetOdometry(t1.getInitialPose(), m_drive),
            new IndexerIntakeActive(m_indexer, m_intake).withTimeout(0.4),
            new ParallelRaceGroup(
                new IndexerIntakeActive(m_indexer, m_intake),
                r1
            ),
            new IndexerIntakeActive(m_indexer, m_intake).withTimeout(0.5),
            // new ConfigureProperty<>(shooterSweepDirection, SweepDirection.kRight),
            new RunIndexerBack(m_intake, m_indexer).withTimeout(Shooter.ARMING_TIME),
            new ParallelCommandGroup(
                new OperateShooter(m_limelight, m_turret, m_flywheel, m_indexer, m_shooterSweepDirection, m_shooterConfiguration, m_shooterOffset).withTimeout(2),
                new ResetOdometry(t2.getInitialPose(), m_drive)
            ),
            new ParallelRaceGroup(
                new IndexerIntakeActive(m_indexer, m_intake),
                r2
            ),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new IndexerIntakeActive(m_indexer, m_intake).withTimeout(1.5),
                    new RunIndexerBack(m_intake, m_indexer).withTimeout(Shooter.ARMING_TIME)
                ),
                new ResetOdometry(t3.getInitialPose(), m_drive),
                new ConfigureProperty<>(shooterSweepDirection, SweepDirection.kRight)
            ),
            r3, 
            new OperateShooter(m_limelight, m_turret, m_flywheel, m_indexer, m_shooterSweepDirection, m_shooterConfiguration, m_shooterOffset).withTimeout(2)

            // V2: run intake after got to the human station
            // new SlowGear(m_drive),
            // new ResetOdometry(t1.getInitialPose(), m_drive),
            // new ParallelCommandGroup(
            //     new SequentialCommandGroup(
            //         new IndexerIntakeActive(m_indexer, m_intake).withTimeout(0.5),
            //         new ParallelRaceGroup(
            //             new IndexerIntakeActive(m_indexer, m_intake),
            //             r1
            //         ),
            //         new IndexerIntakeActive(m_indexer, m_intake).withTimeout(0.5),
            //         new RunIndexerBack(m_intake, m_indexer).withTimeout(Shooter.ARMING_TIME)
            //     ),
            //     new ConfigureShooter(m_turret, m_limelight, m_shooterConfiguration, ShooterMode.kFar)
            // ),
            // new ParallelCommandGroup(
            //     new OperateShooter(m_limelight, m_turret, m_flywheel, m_indexer, m_shooterSweepDirection, m_shooterConfiguration, m_shooterOffset).withTimeout(3),
            //     new ResetOdometry(t2.getInitialPose(), m_drive)
            // ),
            // r2,
            // new ParallelCommandGroup(
            //     new SequentialCommandGroup(
            //         new IndexerIntakeActive(m_indexer, m_intake).withTimeout(2.5),
            //         new RunIndexerBack(m_intake, m_indexer).withTimeout(Shooter.ARMING_TIME)
            //     ),
            //     new ResetOdometry(t3.getInitialPose(), m_drive)
            // ),
            // r3, 
            // new OperateShooter(m_limelight, m_turret, m_flywheel, m_indexer, m_shooterSweepDirection, m_shooterConfiguration, m_shooterOffset).withTimeout(3)
        );
    }
}
