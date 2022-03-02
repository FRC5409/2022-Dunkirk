package frc.robot.commands.autonomous.trajectoryAuto;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.kAuto;
import frc.robot.commands.IndexerIntakeActive;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pigeon;

public class ThreeBallsAuto extends SequentialCommandGroup{

    DriveTrain m_drive;
    Pigeon m_pigeon;
    Intake m_intake;
    Indexer m_indexer;

    public ThreeBallsAuto(DriveTrain drive, Pigeon pigeon, Intake intake, Indexer indexer){

        m_drive   = drive;
        m_pigeon  = pigeon;
        m_intake  = intake;
        m_indexer = indexer;

        m_drive.zeroEncoders();

        Trajectory t1 = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
                                                                   List.of(new Translation2d(0.6, 0.5)),
                                                                   new Pose2d(0.85, 0.7, new Rotation2d(Math.PI*5/12)), 
                                                                   kAuto.configNoStop);

        RamseteCommand r1 = new RamseteCommand(t1, m_pigeon::getPose,
        new RamseteController(kAuto.kRamseteB, kAuto.kRamseteZeta),
        new SimpleMotorFeedforward(kAuto.ksVolts, 
                                   kAuto.kvVoltSecondsPerMeter,
                                   kAuto.kaVoltSecondsSquaredPerMeter),
        kAuto.kDriveKinematics, m_drive::getWheelSpeeds,
        new PIDController(kAuto.kPDriveVel, 0, 0), 
        new PIDController(kAuto.kPDriveVel, 0, 0),
        m_drive::tankDriveVolts, 
        m_drive); 

        m_pigeon.resetOdometry(t1.getInitialPose()); //?

        Trajectory t2 = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
                                                                   List.of(new Translation2d(1, 1)),
                                                                   new Pose2d(2, 0, new Rotation2d(0)), 
                                                                   kAuto.configNoStop);

        RamseteCommand r2 = new RamseteCommand(t2, m_pigeon::getPose,
        new RamseteController(kAuto.kRamseteB, kAuto.kRamseteZeta),
        new SimpleMotorFeedforward(kAuto.ksVolts, 
                                    kAuto.kvVoltSecondsPerMeter,
                                    kAuto.kaVoltSecondsSquaredPerMeter),
        kAuto.kDriveKinematics, m_drive::getWheelSpeeds,
        new PIDController(kAuto.kPDriveVel, 0, 0), 
        new PIDController(kAuto.kPDriveVel, 0, 0),
        m_drive::tankDriveVolts, 
        m_drive); 

        Trajectory t3 = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
                                                                   List.of(new Translation2d(1, 1)),
                                                                   new Pose2d(2, 0, new Rotation2d(0)), 
                                                                   kAuto.configNoStop);

        RamseteCommand r3 = new RamseteCommand(t3, m_pigeon::getPose,
        new RamseteController(kAuto.kRamseteB, kAuto.kRamseteZeta),
        new SimpleMotorFeedforward(kAuto.ksVolts, 
                                    kAuto.kvVoltSecondsPerMeter,
                                    kAuto.kaVoltSecondsSquaredPerMeter),
        kAuto.kDriveKinematics, m_drive::getWheelSpeeds,
        new PIDController(kAuto.kPDriveVel, 0, 0), 
        new PIDController(kAuto.kPDriveVel, 0, 0),
        m_drive::tankDriveVolts, 
        m_drive); 

        Trajectory t4 = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
                                                                   List.of(new Translation2d(1, 1)),
                                                                   new Pose2d(2, 0, new Rotation2d(0)), 
                                                                   kAuto.configStop);

        RamseteCommand r4 = new RamseteCommand(t4, m_pigeon::getPose,
        new RamseteController(kAuto.kRamseteB, kAuto.kRamseteZeta),
        new SimpleMotorFeedforward(kAuto.ksVolts, 
                                    kAuto.kvVoltSecondsPerMeter,
                                    kAuto.kaVoltSecondsSquaredPerMeter),
        kAuto.kDriveKinematics, m_drive::getWheelSpeeds,
        new PIDController(kAuto.kPDriveVel, 0, 0), 
        new PIDController(kAuto.kPDriveVel, 0, 0),
        m_drive::tankDriveVolts, 
        m_drive); 

        addCommands(
            // shoot command
            r1,
            new ParallelCommandGroup().alongWith(
                new IndexerIntakeActive(m_indexer, m_intake).withTimeout(1.5),
                r2
            ),
            r3,
            new ParallelCommandGroup().alongWith(
                new IndexerIntakeActive(m_indexer, m_intake).withTimeout(1.5),
                r4
            )
            // shoot command
        );
    }
}