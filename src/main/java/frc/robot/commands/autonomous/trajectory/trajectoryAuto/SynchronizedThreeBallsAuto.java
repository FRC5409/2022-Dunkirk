package frc.robot.commands.autonomous.trajectory.trajectoryAuto;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.kAuto;
import frc.robot.base.Property;
import frc.robot.base.ValueProperty;
import frc.robot.base.command.DelayedCancelCommand;
import frc.robot.base.command.DelayedCommand;
import frc.robot.base.command.ProxySequentialCommandGroup;
import frc.robot.base.shooter.ShooterConfiguration;
import frc.robot.base.shooter.ShooterMode;
import frc.robot.base.shooter.SweepDirection;
import frc.robot.commands.ConfigureProperty;
import frc.robot.commands.SlowGear;
import frc.robot.commands.autonomous.trajectory.ResetOdometry;
import frc.robot.commands.indexer.IndexerIntakeActive;
import frc.robot.commands.shooter.ConfigureShooter;
import frc.robot.commands.shooter.OperateShooterDelayed;
import frc.robot.commands.shooter.PrimeShooter;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.shooter.ShooterFlywheel;
import frc.robot.subsystems.shooter.ShooterTurret;

public class SynchronizedThreeBallsAuto extends ProxySequentialCommandGroup {

    DriveTrain drive;
    Intake intake;
    Indexer indexer;
    Limelight limelight;
    ShooterTurret turret;
    ShooterFlywheel flywheel;
    Property<ShooterConfiguration> shooterConfiguration;
    Property<SweepDirection> shooterSweepDirection;
    Property<Integer> shooterOffset;

    public SynchronizedThreeBallsAuto(
        DriveTrain drive, 
        Intake intake, 
        Indexer indexer, 
        Limelight limelight, 
        ShooterTurret turret, 
        ShooterFlywheel shooterFlywheel, 
        Property<ShooterConfiguration> shooterConfiguration, 
        Property<SweepDirection> shooterSweepDirection,
        Property<Integer> shooterOffset,
        Property<Boolean> shooterArmed
    ) {

        this.drive   = drive;
        this.intake  = intake;
        this.indexer = indexer;
        this.limelight = limelight;
        this.turret = turret;
        this.flywheel = shooterFlywheel;
        this.shooterConfiguration = shooterConfiguration;
        this.shooterSweepDirection = shooterSweepDirection;
        this.shooterOffset = shooterOffset;

        /*
        Adjustment of robot position at the terminal

        forward x meter: +x*Math.sin(Math.PI*13/36), +x*Math.cos(Math.PI*13/36)
        backward x meter: -x*Math.sin(Math.PI*13/36), -x*Math.cos(Math.PI*13/36)
        left x meter: -x*Math.cos(Math.PI*13/36), +x*Math.sin(Math.PI*13/36)
        right x meter: +x*Math.cos(Math.PI*13/36), -x*Math.sin(Math.PI*13/36)
        */
        
        Trajectory t1 = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
                                                                   List.of(),
                                                                   new Pose2d(1.5/kAuto.kDistanceRatio, 0, new Rotation2d(0)),
                                                                   kAuto.configForwards);

        Trajectory t2 = TrajectoryGenerator.generateTrajectory(new Pose2d(1.5/kAuto.kDistanceRatio, 0, new Rotation2d(0)),
                                                                   List.of(),
                                                                   new Pose2d((5.14+0.75*Math.cos(Math.PI*13/36))/kAuto.kDistanceRatio, (-0.45-0.75*Math.sin(Math.PI*13/36))/kAuto.kDistanceRatio, new Rotation2d(Math.PI*5/36)),
                                                                   kAuto.configForwards);

        Trajectory t3 = TrajectoryGenerator.generateTrajectory(new Pose2d((5.14+0.75*Math.cos(Math.PI*13/36))/kAuto.kDistanceRatio, (-0.45-0.75*Math.sin(Math.PI*13/36))/kAuto.kDistanceRatio, new Rotation2d(Math.PI*5/36)),
                                                                   List.of(),
                                                                   new Pose2d(1.7/kAuto.kDistanceRatio, 0, new Rotation2d(0)),
                                                                   kAuto.configBackwards);

        RamseteCommand r1 = createRamseteCommand(t1);
        RamseteCommand r2 = createRamseteCommand(t2);
        RamseteCommand r3 = createRamseteCommand(t3);

        Property<Boolean> runShooter = new ValueProperty<>(false);
        Trigger runShooterTrigger = new Trigger(runShooter::get);

        drive.setBrakeMode(true);

        Command runShooterCommand1 = new OperateShooterDelayed(limelight, turret, flywheel, indexer, shooterSweepDirection, shooterConfiguration, shooterOffset, shooterArmed, runShooterTrigger);
        Command runShooterCommand2 = new OperateShooterDelayed(limelight, turret, flywheel, indexer, shooterSweepDirection, shooterConfiguration, shooterOffset, shooterArmed, runShooterTrigger);

        addCommands(
            new ConfigureShooter(turret, limelight, shooterConfiguration, ShooterMode.kFar),

            // V1: run intake while getting to the human station
            new SlowGear(drive),

            // Reset Odometry to initial position
            new ResetOdometry(t1.getInitialPose(), drive),


            new ParallelCommandGroup(
                // Run indexer, while moving trough trajectory #2
                new IndexerIntakeActive(indexer, intake).withTimeout(2),
                new SequentialCommandGroup(
                    new WaitCommand(0.35),
                    r1 // Race Condition
                ),
                
                // Schedule shooter  
                new ScheduleCommand(
                    new DelayedCommand(1.5, 
                        new SequentialCommandGroup(
                            new ConfigureProperty<>(shooterSweepDirection, SweepDirection.kLeft),
                            new ConfigureProperty<>(shooterArmed, false),
                            runShooterCommand1
                        )
                    )
                )
            ),

            // Run indexer for additional time
            new IndexerIntakeActive(indexer, intake).withTimeout(0.5),
            
            // Prime shooter
            new PrimeShooter(indexer, shooterArmed).withTimeout(Constants.Shooter.ARMING_TIME),

            // Simulate run shooter trigger
            new ConfigureProperty<>(runShooter, true),

            // Shooter is shooting (eiow)
            // ...

            // Stop shooter from shooting after one second
            new DelayedCancelCommand(2, runShooterCommand1),
            
            // Reset Odometry to next position
            new ResetOdometry(t2.getInitialPose(), drive),
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
                        new WaitCommand(1.5), // TODO: change this value smaller
                        // Reset Odometry to next position
                        new ResetOdometry(t3.getInitialPose(), drive)
                    ),

                    // Run indexer and intake, while moving through trajectory #2
                    new ParallelCommandGroup(
                        r3,
                        // Schedule shooter  
                        new ScheduleCommand(
                            new SequentialCommandGroup(
                                new ConfigureProperty<>(shooterSweepDirection, SweepDirection.kRight),
                                new ConfigureProperty<>(shooterArmed, false),
                                runShooterCommand2
                            )
                        )
                    )
                )
            ),
            // Prime shooter
            new PrimeShooter(indexer, shooterArmed).withTimeout(Constants.Shooter.ARMING_TIME),

            // Simulate run shooter trigger
            new ConfigureProperty<>(runShooter, true),

            // Stop shooter from shooting after one second
            new DelayedCancelCommand(2, runShooterCommand2)

            // // Finish

        );
    }

    private RamseteCommand createRamseteCommand(Trajectory trajectory) {
        return new RamseteCommand(trajectory, drive::getPose,
            new RamseteController(kAuto.kRamseteB, kAuto.kRamseteZeta),
            new SimpleMotorFeedforward(
                kAuto.ksVolts, 
                kAuto.kvVoltSecondsPerMeter,
                kAuto.kaVoltSecondsSquaredPerMeter
            ),
            kAuto.kDriveKinematics,
            drive::getWheelSpeeds,
            new PIDController(kAuto.kPDriveVel, 0, 0), 
            new PIDController(kAuto.kPDriveVel, 0, 0),
            drive::tankDriveVolts, 
            drive
        ); 
    }
}
