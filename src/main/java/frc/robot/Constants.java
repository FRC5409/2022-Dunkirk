// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static class Pneumatics {
        public static final int MODULE = 1;
        public static final double MIN_PSI = 110;
        public static final double MAX_PSI = 120;

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

    public static final class kIntakeIndexer{
        public static final int kIntakeMotor = 4;

        public static final int kRightIntakeSolenoid1 = 1;
        public static final int kRightIntakeSolenoid2 = 0;

        public static final int kLeftIntakeSolenoid1 = 3;
        public static final int kLeftIntakeSolenoid2 = 2;

        public static final int velocityMaxIntakeJam = 1000;
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
        public static final int ForwardChannel = 9;
        public static final int ReverseChannel = 8;

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

    }

    public final class kGyroSystem{

        public static final int CANPigeon = 0;

    }
}
