package frc.robot.commands.autonomous.trajectoryAuto;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.kAuto;
import frc.robot.base.Property;
import frc.robot.base.shooter.ShooterConfiguration;
import frc.robot.base.shooter.ShooterMode;
import frc.robot.base.shooter.SweepDirection;
import frc.robot.commands.IndexerIntakeActive;
import frc.robot.commands.ResetOdometry;
import frc.robot.commands.RunIndexerBack;
import frc.robot.commands.SlowGear;
import frc.robot.commands.shooter.ConfigureShooter;
import frc.robot.commands.shooter.OperateShooter;
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

        String t1JSON = "paths/Path1.wpilib.json";
        Trajectory t1 = new Trajectory();

        try {
            Path t1Path = Filesystem.getDeployDirectory().toPath().resolve(t1JSON);
            t1 = TrajectoryUtil.fromPathweaverJson(t1Path);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + t1JSON, ex.getStackTrace());
        }

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

        String t2JSON = "paths/Path2.wpilib.json";
        Trajectory t2 = new Trajectory();

        try {
            Path t2Path = Filesystem.getDeployDirectory().toPath().resolve(t2JSON);
            t2 = TrajectoryUtil.fromPathweaverJson(t2Path);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + t2JSON, ex.getStackTrace());
        }

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

        addCommands(
            // testing code (just for trajectory)
            new ResetOdometry(t1.getInitialPose(), m_drive),
            new SlowGear(m_drive),
            r1, 
            new ResetOdometry(t2.getInitialPose(), m_drive),
            r2

            // testing code (without intake)
            // new ResetOdometry(t1.getInitialPose(), m_drive),
            // new SlowGear(m_drive),
            // new ParallelCommandGroup(
            //     new SequentialCommandGroup(
            //         // new IndexerIntakeActive(m_indexer, m_intake).withTimeout(0.5),
            //         r1,
            //         new RunIndexerBack(m_intake, m_indexer).withTimeout(0.3)
            //     ),
            //     new ConfigureShooter(m_turret, m_limelight, m_shooterConfiguration, ShooterMode.kFar)
            // ),
            // new OperateShooter(m_limelight, m_turret, m_flywheel, m_indexer, m_shooterSweepDirection, m_shooterConfiguration, m_shooterOffset).withTimeout(3),
            // new ResetOdometry(t2.getInitialPose(), m_drive),
            // r2,
            // new RunIndexerBack(m_intake, m_indexer).withTimeout(0.3),
            // // TODO: configure shooter again or no?
            // new OperateShooter(m_limelight, m_turret, m_flywheel, m_indexer, m_shooterSweepDirection, m_shooterConfiguration, m_shooterOffset).withTimeout(3)

            // actual code
            // new ResetOdometry(t1.getInitialPose(), m_drive),
            // new SlowGear(m_drive),
            // new ParallelCommandGroup(
            //     new SequentialCommandGroup(
            //         // new IndexerIntakeActive(m_indexer, m_intake).withTimeout(0.5),
            //         new ParallelRaceGroup(
            //             new IndexerIntakeActive(m_indexer, m_intake),
            //             r1
            //         ),
            //         new IndexerIntakeActive(m_indexer, m_intake).withTimeout(0.5),
            //         new RunIndexerBack(m_intake, m_indexer).withTimeout(0.3)
            //     ),
            //     new ConfigureShooter(m_turret, m_limelight, m_shooterConfiguration, ShooterMode.kFar)
            // ),
            // new OperateShooter(m_limelight, m_turret, m_flywheel, m_indexer, m_shooterSweepDirection, m_shooterConfiguration, m_shooterOffset).withTimeout(3),
            // new ResetOdometry(t2.getInitialPose(), m_drive),
            // new ParallelRaceGroup(
            //     new IndexerIntakeActive(m_indexer, m_intake),
            //     r2
            // ),
            // new IndexerIntakeActive(m_indexer, m_intake).withTimeout(0.5),
            // new RunIndexerBack(m_intake, m_indexer).withTimeout(0.3),
            // // TODO: configure shooter again or no?
            // new OperateShooter(m_limelight, m_turret, m_flywheel, m_indexer, m_shooterSweepDirection, m_shooterConfiguration, m_shooterOffset).withTimeout(3)
        );

        // @Override
        // public void initialize(){
        //     try {
        //         Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        //         trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        //     } catch (IOException ex) {
        //         DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        //     }
        // }
    }
}