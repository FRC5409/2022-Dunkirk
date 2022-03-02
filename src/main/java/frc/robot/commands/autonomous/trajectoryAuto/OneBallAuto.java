package frc.robot.commands.autonomous.trajectoryAuto;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.kAuto;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Pigeon;

public class OneBallAuto extends SequentialCommandGroup{

    DriveTrain m_drive;
    Pigeon m_pigeon;
    Indexer m_indexer;

    public OneBallAuto(DriveTrain drive, Pigeon pigeon, Indexer indexer){

        m_drive   = drive;
        m_pigeon  = pigeon;
        m_indexer = indexer;

        m_drive.zeroEncoders();

        Trajectory t1 = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
                                                                   List.of(),
                                                                   new Pose2d(1, 0, new Rotation2d(0)), 
                                                                   kAuto.configStop);

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

        addCommands(
            r1
            // shoot command
        ); // where set volts 0?
    }
}