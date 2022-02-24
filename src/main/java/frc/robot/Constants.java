// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
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
        public static final double MAX_PSI = 110;

        public final int Pigeon = 23;
    }

    public final class Climber {
        public static final int mot_port = 0;

        public static final int DIRECTION_EXTEND = 0;
        public static final int DIRECTION_RETRACT = 1;
        //TODO: determine the extension length
        public static final double EXTENSION_LENGTH = 0;
        public static final double RETRACTION_LENGTH = 0;

        public static final double ARM_SPEED = 1.8;

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

        // Anti-tip compensation
        public static final double pitchCompensation = 0;
        public static final double rollCompensation = 0;

        public static final boolean startWithAntiTip = true;
        public static final double wheelSeparation = 0;

        public static final boolean CounterClockwise = false;
        public static final boolean Clockwise = true;

        public static final double MAX_RPM_FOR_LOW_GEAR = 1000;
    }


    

    public static final class kAuto{
        public static final double kTrackwidthMeters = Units.inchesToMeters(26.25);
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

        // robot characterization
        public static final double ksVolts = 0.5788;
        public static final double kvVoltSecondsPerMeter = 4.1279*Math.pow(10, -6);
        public static final double kaVoltSecondsSquaredPerMeter = 2.0127*Math.pow(10, -7);
        public static final double kPDriveVel = 8.1401*Math.pow(10, -6);
        
        // all units in meters and seconds: max speed & acceleration 3
        public static final double kMaxSpeed = 2;
        public static final double kMaxAcceleration = 2;

        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        // works for most robots, if needs to be tuned: 
        // https://docs.wpilib.org/en/latest/docs/software/advanced-controls/trajectories/ramsete.html#constructing-the-ramsete-controller-object
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

    }

    public static final class ShooterFlywheel {
        //in RPM
        public static final int SHOOTER_TOLERANCE = 240;
        public static final int FEEDER_TOLERANCE = 100;
        public static final int rpmTolerance = 1;

        public static final Gains FEEDER_GAINS = new Gains(0.0001, 0.0, 0.0, 0.00017,0,0);
        public static final Gains UPPER_GAINS = new Gains(0.29873, 0, 0, 0.044265063,0,0);
        public static final Gains LOWER_GAINS = new Gains(0.011, 0, 0, 0.047293811,0,0);
    }


    public final static class Turret {
        //Ratio including gearbox 
        //126 : 1
        public static final double GEAR_RATIO          = 126;

        // Height in meters
        public static final double ROBOT_HEIGHT        = 4;
        public static final double FIXED_ANGLE         = 45;
        public static final Range  LIMITS              = new Range(-10, 20);
        public static final double ALIGNMENT_THRESHOLD = 0.14;

        //TODO fill in pneumatics constants.
        public static final int HOOD_FORWARD_CHANNEL = 13;
        public static final int HOOD_REVERSE_CHANNEL = 12;
    }

    public final class Falcon500 {
        public static final double unitsPerRotation = 2048;
    }

    public final class kColour {
        public static final int proximityThreshold = 100;
    }

    public final class kIndexer {
        public static final int kIndexerMotor = 16; 
        public static final int currentLimit = 20; 

        public static final int kIndexShooterMotor = 30;

        public static final int TOF_Ent = 15; 
        public static final int TOF_Ball1 = 16; 
        public static final int TOF_Ext = 17; 

        public static final int sampleTime = 24; 

        public static final int rangeEnter_1 =  40; 
        public static final int rangeEnter_2 = 105; 

        public static final int rangeBall1_1 = 160; 
        public static final int rangeBall1_2 = 150; 

        public static final int rangeExit_1 = 140; 
        public static final int rangeExit_2 = 180;
     
        public static final int indexerMotorBottom = 8; 
    }

    public static final class Limelight {
        public static final String NETWORK_TABLE_NAME = "limelight";
    }

    public static final class Training {
        public static final Range  DISTANCE_RANGE = new Range(0.0, 20);
        public static final String TRAINER_HOSTNAME = "10.54.9.150";
    }
    
    public static final class Shooter {
        public static final double GEAR_RATIO          = 126;

        // Height in meters
        public static final double ROBOT_HEIGHT        = 4;
        public static final double FIXED_ANGLE         = 45;
        public static final Range  TARGET_RANGE        = new Range(-10, 20);
        public static final double ALIGNMENT_THRESHOLD = 0.14;
        public static final double TURRET_MAX_SPEED    = 0.42;

        // Range Configurations
        public static final Range ROTATION_RANGE = new Range(
            -28.571428571428573, 57.14285714285714
        );

        public static final Range SPEED_RANGE = new Range(
            0, 5500
        );

        public static final Range DISTANCE_RANGE = new Range(
            0, 25
        );

        
    // Curve fitting Constants
        public static final Equation DISTANCE_SPEED_CURVE = d -> {
            return d*0;
        };

        public static final double CALIBRATE_SPEED = 0.07;


    // Smooth Sweep Constants (experimental)
        public static final double SHOOTER_SWEEP_PERIOD = 3.6;

        public static final Equation SHOOTER_SWEEP_FUNCTION = new Equation() {
            private final double kA = (2.0 * Math.PI) / SHOOTER_SWEEP_PERIOD;
            private final double kB = 1.0 / ( 2.0 * ROTATION_RANGE.magnitude() );
            private final double kC = ROTATION_RANGE.min();
            
            @Override
            public double calculate(double x) {
                //return (Math.cos(kA * x) + 1.0) * kB + kC;
                return (Math.cos(2d*Math.PI*x/SHOOTER_SWEEP_PERIOD)+1d)/2d*ROTATION_RANGE.magnitude()+ROTATION_RANGE.min();
            }
        };

        public static final Equation SHOOTER_SWEEP_INVERSE = new Equation() {
            private final double kA = 2.0 * (1.0 / ROTATION_RANGE.magnitude());
            private final double kB = 1.0 / ( 2.0 * Math.PI );
            private final double kC = ROTATION_RANGE.min();
            
            @Override
            public double calculate(double x) {
                //return SHOOTER_SWEEP_PERIOD * Math.acos(kA * (x-kC) - 1.0) * kB;
                 return SHOOTER_SWEEP_PERIOD * Math.acos(2d * (x-ROTATION_RANGE.min()) / ROTATION_RANGE.magnitude() - 1d) / (Math.PI*2d);
            }
        };
        
        public static final double SHOOTER_MAX_SWEEEP = 2;
    
        public static final Gains TURRET_GAINS = new Gains(
            /*0.35d*/ 0.15d, 0.0, 1.852d, 0,0,0
        );

        public static final double ALIGNMENT_MAX_TIME = 2;

        public static final double PRE_SHOOTER_DISTANCE = 0;
    }
    
    public static final class Vision {
        public static final double TARGET_HEIGHT = 104d/12.0d;
        
        public static final double LIMELIGHT_HEIGHT = 39.5d/12.0d;

        public static final double LIMELIGHT_PITCH = 90 - 78.2;//13.4;//13.15

        public static final double ACQUISITION_DELAY = 0.35;

        public static final double ALIGNMENT_THRESHOLD = 1.13333;

        protected static double DISTANCE_OFFSET = - 2.0;

        public static final Equation DISTANCE_FUNCTION = new Equation() {
            private final double height = Math.abs(TARGET_HEIGHT - LIMELIGHT_HEIGHT);
            @Override
            public double calculate(double x) {
                return height / Math.tan(Math.toRadians(x + Constants.Vision.LIMELIGHT_PITCH)) + Constants.Vision.DISTANCE_OFFSET;
            }
            
        };

        public static final double ROTATION_P = 0.10;
    }
}