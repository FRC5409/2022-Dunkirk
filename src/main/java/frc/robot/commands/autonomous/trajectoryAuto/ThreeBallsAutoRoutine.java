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
import frc.robot.Constants.kDriveTrain;
import frc.robot.commands.IntakeActive;
import frc.robot.commands.ReverseIntakeIndexer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pigeon;

public class ThreeBallsAutoRoutine extends SequentialCommandGroup{

    DriveTrain m_drive;
    Pigeon m_pigeon;
    Intake m_intake;
    Indexer m_indexer;

    public ThreeBallsAutoRoutine(DriveTrain drive, Pigeon pigeon, Intake intake, Indexer indexer){

        m_drive   = drive;
        m_pigeon  = pigeon;
        m_intake  = intake;
        m_indexer = indexer;

        m_drive.zeroEncoders();

        Trajectory t1 = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
                                                                   List.of(new Translation2d(1, 1)),
                                                                   new Pose2d(2, 0, new Rotation2d(0)), 
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

        m_pigeon.resetOdometry(t1.getInitialPose());

        Trajectory t2 = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
                                                                   List.of(new Translation2d(1, 1)),
                                                                   new Pose2d(2, 0, new Rotation2d(0)), 
                                                                   kAuto.configStop);

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

        addCommands(
            r1,
            new ParallelCommandGroup().alongWith(
                new IntakeActive(m_intake, m_indexer).withTimeout(1),
                r2
            ),
            new ReverseIntakeIndexer(intake, indexer).withTimeout(1)
        ); // where set volts 0?
    }
}