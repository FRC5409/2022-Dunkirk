// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.trajectory.TrajectoryConfig;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import frc.robot.base.shooter.ShooterConfiguration;
import frc.robot.base.shooter.ShooterConfigurationProvider;
import frc.robot.base.shooter.VisionPipeline;
import frc.robot.base.shooter.odometry.ShooterOdometryModel;
import frc.robot.base.shooter.odometry.ShooterTrackingModel;
import frc.robot.base.shooter.target.FilterFactory;
import frc.robot.base.shooter.target.TargetFiltering;
import frc.robot.base.Model3;
import frc.robot.base.Model4;
import frc.robot.base.shooter.HoodPosition;
import frc.robot.base.shooter.ShooterMode;
import frc.robot.base.shooter.TrackingGains;
import frc.robot.utils.*;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class General {
        public enum Configuration {
            kCompetition, kTraining, kTest
        }

        public static final Configuration ROBOT_CONFIGURATION = Configuration.kCompetition;
    }

    public static class kConfig {
        public final static boolean DEBUG = false;
    }

    public static class kID{
        // Can IDs

        public final static int LeftFrontDrive = 1;
        public final static int LeftRearDrive = 2;
        public final static int RightFrontDrive = 3;
        public final static int RightRearDrive = 4;

        public final static int CANCoderLeft = 5;
        public final static int CANCoderRight = 6;

        public final static int IntakeFalcon = 7;
        public final static int IndexNeo1 = 8;
        public final static int IndexNeo2 = 9;
        public final static int IndexToFBottom = 10;
        public final static int IndexToFTop = 11;
        
        public final static int TurretNeo = 12;
        public final static int TurretLimitSwitch1 = 1;
        public final static int ShooterFalconMotor1 = 5;
        public final static int ShooterFalconMotor2 = 8;
        public final static int ShooterNeo = 9;

        public final static int ClimberToF1 = 18;
        public final static int ClimberToF2 = 19;
        public static final int ClimberTofMain = 10;
        public final static int ClimberFalcon1 = 20;
        public final static int ClimbeFalcon2 = 21;
        public final static int PneumaticHub = 2;

        public final static int Pigeon = 23;
    }

    public static class PneumaticsIDs{
        public final int GearShiftHigh  = 9;
        public final int GearShiftLow   = 8;
        public final int IntakeRightIn  = 14;
        public final int IntakeRightOut = 1;
        public final int IntakeLeftIn   = 13;
        public final int IntakeLeftOut  = 2;
        public final int ElevatorUnlock = 12;
        public final int ElevatorLock   = 3;
    }

    public static class kPneumatics {
        public static final double MIN_PSI = 90;
        public static final double MAX_PSI = 115;

        public final int Pigeon = 23;
    }

    public static enum PneumaticsStates {
        kStarted, kEnded, kAny
    }

    public static enum ClimberDestination {
        lockLow,
        lockMid,

        lowRung,
        midRung,
        
        slider
    };

    public final class kClimber {

        public static final int CAN_MASTER_MOT = 20;
        public static final int CAN_FOLLOWER_MOT = 21;

        public static final int DIRECTION_RETRACT = -1;
        public static final int DIRECTION_STATIONARY = 0; 
        public static final int DIRECTION_EXTEND = 1;

        public static final double EXTENSION_LENGTH = 106.5;
        public static final double RETRACTION_LENGTH = 0.5;

        public static final double ARM_SPEED = 1.8;

        public static final double GEAR_RATIO = 20;

        public static final double CIRCUMFERENCE = 0.11427;

        // TODO: Find PIDF values
        public static final double P = 0.8;
        public static final double I = 0;
        public static final double D = 1.7;
        public static final double F = 0;
        public static final int DIGITAL_INPUT_PORT = 8;

        public static final double TO_MAX = 106.5;
        public static final double TO_MID_RUNG = 104.0;
        public static final double TO_LOW_RUNG = 55.0;
        public static final double TO_MIN_MID = 10.0;
        public static final double TO_MIN_LOW = 4.0;
    }

    public static final class kIntake{
        public static final int kIntakeMotor = 11;

        public static final int kIntakeMotorIn = 24; 


        public static final int kRightIntakeSolenoid1 = 11;
        public static final int kRightIntakeSolenoid2 = 10;

        public static final int kLeftIntakeSolenoid1 = 4;
        public static final int kLeftIntakeSolenoid2 = 5;

        public static final int velocityMaxIntakeJam = 1000;
    }

    public static final class kDriveTrain{
        
        // Current Limits
        public static final double CurrentLimit = 65;
        public static final double TriggerThresholdCurrent = 65;
        public static final double triggerThresholdTime = 0;

        // Encoders
        public static final double encoderToMeterConversionFactor = 1;

        public static final double encoderCPR = 2048;
        public static final double wheelCircumferenceInches = 4 * Math.PI;
        public static final double lowGearConversionFactor = 1/15.32;
        public static final double highGearConversionFactor = 1/7.08;

        // Double Solenoid
        public static final int ForwardChannel = 8;
        public static final int ReverseChannel = 9;

        // Drive Modes
        public static final int InitialDriveMode = 0;

        public static final int AADIL_DRIVE = 0;
        public static final int TANK_DRIVE = 1;
        public static final double MAX_RPM_LOW_GEAR = 1000;

        // PID Controls
        public static final int kSlotIdx = 0;
        public static final int kPIDLoopIdx = 0;
        public static final int kTimeoutMs = 30;
        public static final boolean kSensorPhase = true;
        
        public static final Gains kDistanceGains = new Gains(0.15, 0.0002, 0.1, 0.0, 0, 1.0);
        public static final Gains kAngleGains = new Gains(0.15, 0.0, 0.1, 0.0, 0, 1.0);
        
        // Speed limits for auto
        public static final double maxStraightSpeed = 1;
        public static final double maxTurnSpeed = 1;

        // pigeon mount position TODO: TUNE THESE VALUES
        public static final double gyroMountYaw   = 0;
        public static final double gyroMountPitch = 0;
        public static final double gyroMountRoll  = 0;

        //
        public static final double forwardRampRate = 0.4;
        public static final double backwardRampRate = 0.3;
        public static final double forwardTurnRampRate = 0.1;
        public static final double backwardTurnRampRate = 0.1;

        // Anti-tip compensation
        public static final double pitchCompensation = 0;
        public static final double rollCompensation = 0;

        public static final boolean startWithAntiTip = true;
        public static final double wheelSeparation = 0;

        public static final boolean CounterClockwise = false;
        public static final boolean Clockwise = true;

        public static final double MAX_RPM_FOR_LOW_GEAR = 1000;
        public static final double DISTANCE_TO_MID_RUN_FROM_WALL = 2.2098;
        public static final double METERS_TO_RSU = 12_832.0802;
    }

    public static final class kAuto{
        public static final double kTrackwidthMeters = 0.78089;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

        // robot characterization
        public static final double ksVolts = 0.61192;
        public static final double kvVoltSecondsPerMeter = 5.041;
        public static final double kaVoltSecondsSquaredPerMeter = 0.31042;
        public static final double kPDriveVel = 5.7255;
        
        // all units in meters and seconds
        public static final double kMaxSpeed = 2.5; 
        // work fine in 2.5, gives error when generating trajectory when exceed that value
        public static final double kMaxAcceleration = 4.5;

        public static final double kMaxSpeedSlow = 0.75;
        public static final double kMaxAccelerationSlow = 0.5;

        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        // works for most robots, if needs to be tuned: 
        // https://docs.wpilib.org/en/latest/docs/software/advanced-controls/trajectories/ramsete.html#constructing-the-ramsete-controller-object
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        public static final double kDistanceRatio = -0.95;

        public static final DifferentialDriveVoltageConstraint autoVoltageConstraint = 
            new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(ksVolts, 
                                           kvVoltSecondsPerMeter, 
                                           kaVoltSecondsSquaredPerMeter),
                kDriveKinematics, 10);

        public static final TrajectoryConfig configForwards = 
            new TrajectoryConfig(kMaxSpeed, kMaxAcceleration)
            .setKinematics(kDriveKinematics)
            .addConstraint(autoVoltageConstraint)
            .setEndVelocity(0)
            .setReversed(true);

        public static final TrajectoryConfig configBackwards = 
            new TrajectoryConfig(kMaxSpeed, kMaxAcceleration)
            .setKinematics(kDriveKinematics)
            .addConstraint(autoVoltageConstraint)    
            .setEndVelocity(0)
            .setReversed(false);

        public static final TrajectoryConfig configBackwardsSlow = 
            new TrajectoryConfig(kMaxSpeedSlow, kMaxAccelerationSlow)
            .setKinematics(kDriveKinematics)
            .addConstraint(autoVoltageConstraint)    
            .setEndVelocity(1)
            .setReversed(false);
    }

    public final class Falcon500 {
        public static final double unitsPerRotation = 2048;
    }

    public final class Indexer {
        public static final int INDEXER_MOTOR_ID = 8;

        public static final int CURRENT_LIMIT = 20; 

        public static final int kIndexShooterMotor = 30;

        public static final int TOF_ENTER_CHANNEL = 25; 
        public static final int TOF_BALL_1_CHANNEL = 26; 
        public static final int TOF_EXIT_CHANNEL = 27; 

        public static final int SAMPLE_TIME = 24; 

        public static final int RANGE_ENTER = 180;
        public static final int RANGE_BALL_1 = 105;
        public static final int RANGE_EXIT = 90; 
     
    }

    public static final class Limelight {
        public static final String NETWORK_TABLE_NAME = "limelight";
    }

    public static final class Training {
        public static final Range  DISTANCE_RANGE = new Range(0.0, 20);
        public static final String TRAINER_HOSTNAME = "10.54.9.150";
        public static final int TRAINER_PORT = 5409;
        public static final FilterFactory<?> DEFAULT_TARGET_FILTER = TargetFiltering.none();
    }
    
    public static final class Shooter {
                // ============================================================== \\
        //               NICOLE TUNE THIS PURR - Keith

        // The shooter Pitch (observed with the inclinometer)
        public static final double SHOOTER_PITCH = 54.8;

        // The initial flywheel offset (rpm)
        public static final double INITIAL_SHOOTER_OFFSET = 200;

        // The turret rotation range
        public static final Range ROTATION_RANGE = new Range(-110, 110);
        
        // The threshold required to align to a target (and shoot) during tracking
        // between [2, 8]
        public static final double TRACKING_ALIGNMENT_THRESHOLD = 22.83333;

        // How long to run the indexer back, and arm the shooter.
        public static final double ARMING_TIME = 0.32;
        

        public static final int FLYWHEEL_TOLERANCE = 175;//25;

        //                                                                //
        // ============================================================== \\
        public static final boolean ZERO_LIMIT_POLARITY = false;

        public static final int HOOD_FORWARD_CHANNEL = 13;
        public static final int HOOD_REVERSE_CHANNEL = 12;

        public static final int FEEDER_TOLERANCE = 150;

        public static final Gains FEEDER_GAINS = new Gains(0.0001, 0.0, 0.0, 0.000188,0,0);
        public static final Gains FLYWHEEL_GAINS = new Gains(0.475, 0, 0, 0.049,0,0);

        public static final double GEAR_RATIO = 280;
            
        // Range Configurations
        public static final Range SPEED_RANGE = new Range(0, 5500);
        public static final Range DISTANCE_RANGE = new Range(0, 25);
        public static final Range TURRET_OUTPUT_RANGE = new Range(-0.8, 0.8);
        
        // Smooth Sweep Constants
        public static final double   SHOOTER_SWEEP_PERIOD = 2.75;
        public static final double   SHOOTER_MAX_SWEEEP = 2;

        public static final Equation SHOOTER_SWEEP_FUNCTION = new Equation() {
            @Override
            public double calculate(double x) {
                return (Math.cos(2d*Math.PI*x/SHOOTER_SWEEP_PERIOD)+1d)/2d*ROTATION_RANGE.magnitude()+ROTATION_RANGE.min();
            }
        };

        public static final Equation SHOOTER_SWEEP_INVERSE = new Equation() {
            @Override
            public double calculate(double x) {
                 return SHOOTER_SWEEP_PERIOD * Math.acos(2d * (x-ROTATION_RANGE.min()) / ROTATION_RANGE.magnitude() - 1d) / (Math.PI*2d);
            }
        };
    
        public static final Gains TURRET_ROTATION_GAINS = new Gains(
            /*0.35d*/ 0.15d, 0.0, 1.852d, 0,0,0
        );
        
        public static final Gains TURRET_TRACKING_GAINS = new Gains(
            /*0.35d*/ 0.08d, 0.0, 0.852d, 0,0,0
        );

        public static final double ALIGNMENT_MAX_TIME = 2;

        public static final double PRE_SHOOTER_VELOCITY = 1200;

        public static final double LOW_FLYWHEEL_VELOCITY = 400;
        public static final double GUARD_FLYWHEEL_VELOCITY = 800;
        public static final double NEAR_FLYWHEEL_VELOCITY = 1702;
  
        public static final ShooterTrackingModel TRACKING_MODEL = new ShooterTrackingModel(
            TargetFiltering.none(), 
            new TrackingGains(-0.95, 0, -24.8, 1.0, -1.0, 2.0, -2.0, 1.8),
            Constants.Vision.TARGET_LOST_TIME,

            0.90,
            0.15,
            
            // flywheel_offset
            new Model3(
                //-0.06823772956504384, -1.3965513908806815, 1.18121626304908,
                1.0169577051439325, -3.803142000618278, 2.147331574023152,
                new Range(-3.0, 3.0),
                new Range(-1200, 1200)
            ),

            // turret_offset
            new Model3(
                0.1357190569153385, -1.3929300904057553, 1.1600000296421455, 
                new Range(-3.0, 3.0),
                new Range(-50, 50)
            )
        );

        public static final ShooterOdometryModel ODOMETRY_MODEL = new ShooterOdometryModel(
            90.0 - SHOOTER_PITCH,
            41.5 / 12.0,
            0.25d,
            new Vector3(7/12, 0, -15/12),
            new Vector2(59.6, 49.7)
        );

        public static final Model4 EXECUTION_MODEL = new Model4(
            -0.010504270903766155,
            0.709503173828125,
            0.007261261343955994,
            0.25371038913726807,
            Constants.Shooter.DISTANCE_RANGE,
            Constants.Shooter.SPEED_RANGE
        );

        public static final ShooterConfigurationProvider CONFIGURATIONS = ShooterConfigurationProvider.of(
            new ShooterConfiguration(
                ShooterMode.kFar,
                HoodPosition.kUp,
                VisionPipeline.FAR_TARGETING
            ),
            
            new ShooterConfiguration(
                ShooterMode.kNear,
                HoodPosition.kDown,
                VisionPipeline.NEAR_TARGETING
            ),
            
            new ShooterConfiguration(
                ShooterMode.kLow,
                HoodPosition.kUp,
                VisionPipeline.DEFAULT
            ),
            
            new ShooterConfiguration(
                ShooterMode.kGuard,
                HoodPosition.kUp,
                VisionPipeline.DEFAULT
            )
        );

        public static final double FEEDER_VELOCITY = -5000;

        public static final int OFFSET_INCREMENT = 25;

        public static final double INDEXER_SPEED = 0.5;
        
        public static final double ALIGNMENT_THRESHOLD = 1.83333;
    }
    
    public static final class Vision {
        public static final double ACQUISITION_DELAY = 0.15;
        public static final double TARGET_LOST_TIME = 0.5;
    }
}