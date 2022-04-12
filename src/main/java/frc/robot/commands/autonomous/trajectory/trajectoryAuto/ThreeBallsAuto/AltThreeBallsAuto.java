package frc.robot.commands.autonomous.trajectory.trajectoryAuto.ThreeBallsAuto;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.kAuto;
import frc.robot.base.Property;
import frc.robot.base.ValueProperty;
import frc.robot.base.command.DelayedCancelCommand;
import frc.robot.base.command.ProxySequentialCommandGroup;
import frc.robot.base.drive.DriveOdometry;
import frc.robot.base.drive.MockDriveOdometry;
import frc.robot.base.indexer.IndexerArmedState;
import frc.robot.base.shooter.ShooterConfiguration;
import frc.robot.base.shooter.ShooterMode;
import frc.robot.base.shooter.ShooterModelProvider;
import frc.robot.base.shooter.ShooterState;
import frc.robot.base.shooter.SweepDirection;
import frc.robot.commands.ConfigureProperty;
import frc.robot.commands.SlowGear;
import frc.robot.commands.autonomous.trajectory.ResetOdometry;
import frc.robot.commands.indexer.IndexerIntakeActive;
import frc.robot.commands.shooter.ComplexOperateShooter;
import frc.robot.commands.shooter.ConfigureShooter;
import frc.robot.commands.shooter.PrimeShooter;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.shooter.ShooterFlywheel;
import frc.robot.subsystems.shooter.ShooterTurret;

/**
 * @author Nicole Jin
 */
public class AltThreeBallsAuto extends ProxySequentialCommandGroup {
    private final DriveTrain drivetrain;

    public AltThreeBallsAuto(
        DriveTrain drivetrain, 
        Intake intake, 
        Indexer indexer, 
        Limelight limelight, 
        ShooterTurret turret, 
        ShooterFlywheel flywheel, 
        ShooterModelProvider shooterModelProvider,
        Property<ShooterConfiguration> shooterConfiguration, 
        Property<SweepDirection> shooterSweepDirection,
        Property<IndexerArmedState> indexerArmedState,
        Property<ShooterState> shooterState,
        Property<Double> shooterOffset
    ) {

        this.drivetrain   = drivetrain;

        /*
        Adjustment of robot position at the terminal

        forward x meter: +x*Math.sin(Math.PI*3/20), +x*Math.cos(Math.PI*3/20)
        backward x meter: -x*Math.sin(Math.PI*3/20), -x*Math.cos(Math.PI*3/20)
        left x meter: -x*Math.cos(Math.PI*3/20), +x*Math.sin(Math.PI*3/20)
        right x meter: +x*Math.cos(Math.PI*3/20), -x*Math.sin(Math.PI*3/20)
        */

        // =====================================================================
        // Poses

        Pose2d p1 = createPose(0, 0, 0);

        Pose2d p2 = createPose(1.35, 0, Math.PI/4);

        Pose2d p3 = createPose(4.13, 6.11, Math.PI*7/20); 

        Pose2d p4 = createPose(2.75, 2, Math.PI*7/20);

        
        // =====================================================================
        // Trajectories

        Trajectory t1 = TrajectoryGenerator.generateTrajectory(
            p1, List.of(), p2, kAuto.configForwards
        );

        Trajectory t2 = TrajectoryGenerator.generateTrajectory(
            p2, List.of(), p3, kAuto.configForwards
        );

        Trajectory t3 = TrajectoryGenerator.generateTrajectory(
            p3, List.of(), p4, kAuto.configBackwards
        );

        // =====================================================================
        // Ramsete Commands

        RamseteCommand r1 = createRamseteCommand(t1);
        RamseteCommand r2 = createRamseteCommand(t2);
        RamseteCommand r3 = createRamseteCommand(t3);
        
        // =====================================================================
        // Shooter Commands

        DriveOdometry mockDrivetrain = new MockDriveOdometry();

        Property<Boolean> runShooter = new ValueProperty<>(false);
        Property<Double> mockDriveSpeed = new ValueProperty<>(0.0);

        Trigger runShooterTrigger = new Trigger(runShooter::get);

        ComplexOperateShooter runShooterCommand = new ComplexOperateShooter(
            flywheel, turret, mockDrivetrain, limelight, indexer, runShooterTrigger, 
            shooterModelProvider, shooterConfiguration, indexerArmedState, 
            shooterSweepDirection, shooterState, shooterOffset, mockDriveSpeed);

        // =====================================================================

        addCommands(

            // // trajectory testing
            // new ResetOdometry(t1.getInitialPose(), drivetrain),
            // r1, 
            // new ResetOdometry(t2.getInitialPose(), drivetrain),
            // r2,
            // new ResetOdometry(t3.getInitialPose(), drivetrain),
            // r3

            new InstantCommand(() -> drivetrain.setBrakeMode(true), drivetrain),

            new ConfigureShooter(turret, limelight, shooterConfiguration, ShooterMode.kFar),

            // V1: run intake while getting to the human station
            new SlowGear(drivetrain),

            // Reset Odometry to initial position
            new ResetOdometry(t1.getInitialPose(), drivetrain),


            new ParallelCommandGroup(
                // Run indexer, while moving trough trajectory #1
                new IndexerIntakeActive(indexer, intake).withTimeout(2),
                new SequentialCommandGroup(
                    new WaitCommand(0.35),
                    r1 // Race Condition
                ),
                
                // Schedule shooter  
                new ScheduleCommand(
                    new SequentialCommandGroup(
                        new WaitCommand(1.5), // Delay
                        new ConfigureProperty<>(shooterSweepDirection, SweepDirection.kLeft),
                        runShooterCommand
                    )
                )
            ),

            // Run indexer for additional time
            new IndexerIntakeActive(indexer, intake).withTimeout(0.75),
            
            // Prime shooter
            new PrimeShooter(indexer, indexerArmedState),

            // Simulate run shooter trigger
            new ConfigureProperty<>(runShooter, true),

            // Shooter is shooting (eiow)
            // ...
            
            // Reset Odometry to next position
            new ResetOdometry(t2.getInitialPose(), drivetrain),

            new WaitCommand(2.0),
            
            new ConfigureProperty<>(runShooter, false),
            
            // Run indexer and intake, while moving through trajectory #2
            new ParallelRaceGroup(
                new IndexerIntakeActive(indexer, intake),
                r2 // Race Condition
            ),
                
            new ParallelCommandGroup(
                // Run indexer for additional time
                new IndexerIntakeActive(indexer, intake).withTimeout(2.0),

                new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new WaitCommand(0.5), // TODO: change this value smaller
                        // Reset Odometry to next position
                        new ResetOdometry(t3.getInitialPose(), drivetrain)
                    ),

                    // Run indexer and intake, while moving through trajectory #2
                    r3
                )
            ),
            // Prime shooter
            new PrimeShooter(indexer, indexerArmedState),

            // Simulate run shooter trigger
            new ConfigureProperty<>(runShooter, true),

            // Stop shooter from shooting after one second
            new DelayedCancelCommand(2, runShooterCommand)

            // // Finish
        );
    }

    private RamseteCommand createRamseteCommand(Trajectory trajectory) {
        return new RamseteCommand(trajectory, drivetrain::getPose,
            new RamseteController(kAuto.kRamseteB, kAuto.kRamseteZeta),
            new SimpleMotorFeedforward(
                kAuto.ksVolts, 
                kAuto.kvVoltSecondsPerMeter,
                kAuto.kaVoltSecondsSquaredPerMeter
            ),
            kAuto.kDriveKinematics,
            drivetrain::getWheelSpeeds,
            new PIDController(kAuto.kPDriveVel, 0, 0), 
            new PIDController(kAuto.kPDriveVel, 0, 0),
            drivetrain::tankDriveVolts, 
            drivetrain
        ); 
    }

    private Pose2d createPose(double x, double y, double rotation) {
        return new Pose2d(x / kAuto.kDistanceRatio, y / kAuto.kDistanceRatio, new Rotation2d(rotation));
    }
}
