// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.Gains;

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
        public final static int TurretLimitSwitch1 = 13;
        public final static int TurretLimitSwitch2 = 14;
        public final static int TurretSwitchLimit3 = 15;
        public final static int ShooterFalconMotor1 = 16;
        public final static int ShooterFalconMotor2 = 17;
        public final static int ClimberToF1 = 18;
        public final static int ClimberToF2 = 19;
        public final static int ClimberFalcon1 = 20;
        public final static int ClimbeFalcon2 = 21;
        public final static int PneumaticHub = 2;

        public final static int Pigeon = 23;
    }

    public static class PneumaticsIDs{
        public final int GearShiftHigh  = 15;
        public final int GearShiftLow   = 0;
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

        public final int LeftFrontDrive = 1;
        public final int LeftRearDrive = 2;
        public final int RightFrontDrive = 3;
        public final int RightRearDrive = 4;

        public final int CANCoderLeft = 5;
        public final int CANCoderRight = 6;

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


        public static final int kRightIntakeSolenoid1 = 2;
        public static final int kRightIntakeSolenoid2 = 3;

        public static final int kLeftIntakeSolenoid1 = 4;
        public static final int kLeftIntakeSolenoid2 = 5;

        public static final int velocityMaxIntakeJam = 1000;

    }

    public static final class kIndexer{
        public static final int currentLimit = 20; 
        public static final int indexerMotorBottom = 8; 

        public static final int TOF_Ent = 15; 
        public static final int TOF_Ball1 = 16; 
        public static final int TOF_Ext = 17; 

        public static final int sampleTime = 24; 

        public static final int rangeEnter = 250; 
        public static final int rangeBall1 = 80; 
        public static final int rangeExt = 100; 
    }


    public static final class kDriveTrain{

        // CAN IDs 
        public static final int CANLeftDriveFront = 1;
        public static final int CANRightDriveFront = 3;
        public static final int CANLeftDriveBack = 2;
        public static final int CANRightDriveBack = 4;
        
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
        public static final int ForwardChannel = 0;
        public static final int ReverseChannel = 1;

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

    public final class kGyroSystem{

        public static final int CANPigeon = 23;

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
}


