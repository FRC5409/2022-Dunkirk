package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Convert;
import frc.robot.utils.MotorUtils;
import frc.robot.utils.Vector3;
import frc.robot.Constants.kID;
import frc.robot.base.drive.DriveOdometry;
import frc.robot.base.drive.PigeonData;
import frc.robot.Constants;
import frc.robot.Constants.kDriveTrain;

public class DriveTrain extends SubsystemBase implements DriveOdometry {
    public static enum DriveMode {
        kAadilDrive("AADIL DRIVE"), 
        kTankDrive("TANK DRIVE");

        private final String label;

        private DriveMode(String label) {
            this.label = label;
        }

        public String label() {
            return this.label;
        }
    }

    public static enum DriveState {
        kForward, kReverse, kForwardTurn, kReverseTurn
    }

    public static enum DriveGear {
        kLowGear, kHighGear
    }

    private final WPI_TalonFX mot_leftFrontDrive;
    private final WPI_TalonFX mot_leftRearDrive;
    private final WPI_TalonFX mot_rightFrontDrive;
    private final WPI_TalonFX mot_rightRearDrive;

    private final WPI_Pigeon2 gyro_pigeon;
    private final DoubleSolenoid dsl_gear;
    private final DifferentialDrive ctr_drive;

    private final DifferentialDriveOdometry odometry;

    private DriveState state;
    private PigeonData data;
    private DriveMode mode;
    private boolean odometryEnabled;

    public DriveTrain() {
        mot_leftFrontDrive = new WPI_TalonFX(kID.LeftFrontDrive);
        mot_leftRearDrive = new WPI_TalonFX(kID.LeftRearDrive);
        mot_rightFrontDrive = new WPI_TalonFX(kID.RightFrontDrive);
        mot_rightRearDrive = new WPI_TalonFX(kID.RightRearDrive);

        dsl_gear = new DoubleSolenoid(kID.PneumaticHub, PneumaticsModuleType.REVPH, 
            kDriveTrain.ForwardChannel, kDriveTrain.ReverseChannel);

        gyro_pigeon = new WPI_Pigeon2(kID.Pigeon);
            gyro_pigeon.reset();
            gyro_pigeon.configMountPose(
                kDriveTrain.gyroMountYaw, kDriveTrain.gyroMountPitch, kDriveTrain.gyroMountRoll);

        ctr_drive = new DifferentialDrive(mot_leftFrontDrive, mot_rightFrontDrive);

        odometryEnabled = false;
        odometry = new DifferentialDriveOdometry(gyro_pigeon.getRotation2d());
        data = new PigeonData();
        mode = kDriveTrain.DEFAULT_DRIVE_MODE;

        setNeutralMode(kDriveTrain.DEFAULT_NEUTRAL_MODE);
        zeroEncoders();
        configMotors();
    }
    /**
     * This method is called once per scheduler run and is used to update smart
     * dashboard data.
     */
    public void periodic() {
        data.update(gyro_pigeon);

        if (odometryEnabled)
            odometry.update(data.getRotation2d(), getEncoderPositionLeft(), getEncoderPositionRight());

        displayTemperatures();
        if (Constants.kConfig.DEBUG)
            displayEncoder();
    }

    /**
     * @param acceleration the robot's forward speed
     * 
     * @param deceleration the robot's backward speed
     * 
     * @param turn         the robot's angular speed about the z axis
     * 
     */
    public void aadilDrive(double acceleration, double deceleration, double turn) {
        double accelerate = (acceleration - deceleration);

        if(accelerate > 0) {
            if (turn == 0)
                setDriveState(DriveState.kForward);
            else
                setDriveState(DriveState.kForwardTurn);
        } else {
            if (turn == 0)
                setDriveState(DriveState.kReverse);
            else
                setDriveState(DriveState.kReverseTurn);
        }
        
        ctr_drive.arcadeDrive(accelerate, turn, true);
    }

    /**
     * @param leftSpeed  the robot's left side speed along the X axis [-1.0, 1.0].
     *                   Forward is positive.
     * 
     * @param rightSpeed the robot's right side speed along the X axis [-1.0, 1.0].
     *                   Forward is positive.
     * 
     */
    public void tankDrive(double leftSpeed, double rightSpeed) {
        ctr_drive.tankDrive(leftSpeed, rightSpeed);
    }

    /**
     * Tank drive that takes voltage inputs
     * 
     * @param voltsLeft  voltage for left wheels
     * @param voltsRight voltage for right wheels
     */
    public void tankDriveVolts(double voltsLeft, double voltsRight) {
        mot_leftFrontDrive.setVoltage(voltsLeft);
        mot_rightFrontDrive.setVoltage(voltsRight);
        ctr_drive.feed();
    }

    public void stopMotors() {
        mot_rightFrontDrive.stopMotor();
        mot_rightRearDrive.stopMotor();
        mot_leftFrontDrive.stopMotor();
        mot_leftRearDrive.stopMotor();
    }

    /**
     * resets the GyroSystem's heading 
     */
    public void resetGyro() {
        gyro_pigeon.reset();
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(pose, data.getRotation2d());
        zeroEncoders();
    }

    /**
     * Sets all encoders to 0
     * 
     * call this function when reading positions relative to the robot's current
     * position.
     * 
     */
    public void zeroEncoders() {
        setEncoderPosition(0, 0);
    }

    /**
     * cycles to the next drive mode
     * 
     * Order:
     * AADIL_DRIVE
     * TANK_DRIVE
     * REPEAT
     */
    public void cycleDriveMode() {
        switch (mode) {
            case kAadilDrive:
                mode = DriveMode.kTankDrive;
                break;
            case kTankDrive:
                mode = DriveMode.kAadilDrive;
                break;
        }

        if (Constants.kConfig.DEBUG)
            System.out.println("Cycling drive mode");
    }

    public void setOdometryEnabled(boolean enabled) {
        odometryEnabled = enabled;
    }

    /**
     * Sets the ramp rate
     * 
     * @param rampRate time to full throttle
     * 
     */
    public void setRampRate(double rampRate) {
        mot_leftFrontDrive.configOpenloopRamp(rampRate);
        mot_rightFrontDrive.configOpenloopRamp(rampRate);
    }
  
    public void setNeutralMode(NeutralMode mode) {
        mot_rightFrontDrive.setNeutralMode(mode);
        mot_rightRearDrive.setNeutralMode(mode);
        mot_leftFrontDrive.setNeutralMode(mode);
        mot_leftRearDrive.setNeutralMode(mode);
    }

    public void setDeadband(double deadband) {
        ctr_drive.setDeadband(deadband);
    }

    /**
     * directly set the drive mode to the specified int
     * 
     */
    public void setDriveMode(DriveMode mode) {
        this.mode = mode;
    }

    /**
     * @param enable if true sets all motors to brake, if false sets all motors to
     *               coast.
     * 
     * @deprecated use {@link #setNeutralMode(NeutralMode)}
     */
    @Deprecated
    public void setBrakeMode(boolean enable) {
        setNeutralMode(enable ? NeutralMode.Brake : NeutralMode.Coast);
    }

    public void setEncoderPosition(double position) {
        setEncoderPosition(position, position);
    }

    public void setEncoderPosition(double leftPosition, double rightPosition) {
        mot_rightFrontDrive.setSelectedSensorPosition(leftPosition);
        mot_rightRearDrive.setSelectedSensorPosition(leftPosition);
        mot_leftFrontDrive.setSelectedSensorPosition(rightPosition);
        mot_leftRearDrive.setSelectedSensorPosition(rightPosition);
    }

    public void setControlMode(TalonFXControlMode mode, double value) {
        setControlMode(mode, value, value);
    }

    public void setControlMode(TalonFXControlMode mode, double leftValue, double rightValue) {
        mot_leftFrontDrive.set(mode, leftValue);
        mot_leftRearDrive.set(mode, leftValue);
        mot_rightFrontDrive.set(mode, rightValue);
        mot_rightRearDrive.set(mode, rightValue);
    }

    public void setGearShift(DriveGear gear) {
        switch (gear) {
            case kHighGear:
                dsl_gear.set(DoubleSolenoid.Value.kForward);
                break;
            case kLowGear:
                dsl_gear.set(DoubleSolenoid.Value.kReverse);
                break;
        }
    }

    public void displayTemperatures() {
        SmartDashboard.putNumber("Temp LF", mot_leftFrontDrive.getTemperature());
        SmartDashboard.putNumber("Temp LB", mot_leftRearDrive.getTemperature());
        SmartDashboard.putNumber("Temp RF", mot_rightFrontDrive.getTemperature());
        SmartDashboard.putNumber("Temp RB", mot_rightRearDrive.getTemperature());
    }

    /**
     * Puts the positions and velocities of the left and right encoders into
     * SmartDashboard
     * 
     */
    public void displayEncoder() {
        SmartDashboard.putNumber("Left Position", getEncoderPositionLeft());
        SmartDashboard.putNumber("Left Velocity", getEncoderVelocityLeft());
        SmartDashboard.putNumber("Left RPM", getRPMLeft());

        SmartDashboard.putNumber("Right Position", getEncoderPositionRight());
        SmartDashboard.putNumber("Right Velocity", getEncoderVelocityRight());
        SmartDashboard.putNumber("Right RPM", getRPMRight());
    }

    /**
     * Puts the Roll-Pitch-Yaw into SmartDashboard 
     * 
     */
    public void displayAngle() {
        SmartDashboard.putNumber("Roll",  getRoll());
        SmartDashboard.putNumber("Pitch", getPitch());
        SmartDashboard.putNumber("Yaw",   getYaw());
    }

    /**
     * Puts the angle and rate into SmartDashboard 
     * 
     */
    public void displayHeading() {
        SmartDashboard.putNumber("Angle", getHeading());
        SmartDashboard.putNumber("Rate", getTurnRate());
        SmartDashboard.putNumber("Rotation2d", gyro_pigeon.getRotation2d().getRadians());
    }

    /**
     * @return the current drive mode as an int
     * 
     */
    public DriveMode getDriveMode() {
        return mode;
    }
  
    /**
     * @return the average position of all four encoders
     * 
     */
    public double getEncoderPosition() {
        return (getEncoderPositionLeft() + getEncoderPositionRight()) / 2;
    }

    /**
     * @return the  position of the left encoders 
     * 
     */
    public double getEncoderPositionLeft() {
        double position = Convert.EncoderUnitsToInches(
            (mot_leftFrontDrive.getSelectedSensorPosition()+mot_leftRearDrive.getSelectedSensorPosition()) / 2.0);

        return Units.inchesToMeters(position);
    }

    /**
     * @return the average position of the right encoders
     * 
     */
    public double getEncoderPositionRight() {
        double position = Convert.EncoderUnitsToInches(
            (mot_rightFrontDrive.getSelectedSensorPosition()+mot_rightRearDrive.getSelectedSensorPosition()) / 2.0);

        return Units.inchesToMeters(position);
    }

    /**
     * @return the average velocity all for encoders
     * 
     */
    public double getEncoderVelocity() {
        return (getEncoderVelocityLeft() + getEncoderVelocityRight()) / 2;
    }

    /**
     * @return the average velocity of the left encoders
     * 
     */
    public double getEncoderVelocityLeft() {
        double velocity = 10 * Convert.EncoderUnitsToInches(
            (mot_leftFrontDrive.getSelectedSensorVelocity() + mot_leftRearDrive.getSelectedSensorVelocity()) / 2.0);

        return Units.inchesToMeters(velocity);
    }

    /**
     * @return the average velocity of the right encoders
     * 
     */
    public double getEncoderVelocityRight() {
        double velocity = 10 * Convert.EncoderUnitsToInches(
            (mot_rightFrontDrive.getSelectedSensorVelocity() + mot_rightRearDrive.getSelectedSensorVelocity()) / 2.0);

        return Units.inchesToMeters(velocity);
    }

    public double getRPM() {
        return (getRPMLeft() + getRPMRight()) / 2.0;
    }

    public double getRPMRight() {
        return (getEncoderVelocityRight() / 2048) * 600;
    }

    /**
     * @return the average velocity of the right encoders
     * 
     */
    public double getRPMLeft() {
        return (getEncoderVelocityLeft() / 2048) * 600;
    }

    /**
     * @return double the GyroSystem's roll
     */
    public double getRoll() {
        return data.getRoll();
    }
    
    /**
     * @return double the GyroSystem's yaw
     */
    public double getYaw() {
        return data.getYaw();
    }

    /**
     * @return double the GyroSystem's pitch
     */
    public double getPitch() {
        return data.getPitch();
    }

    /**
     * @return double the heading in degrees. Positive is clockwise.
     *  
     * Note:
     *  - The heading of the robot
     * 
     *  - Follows North-East-Down convention
     * 
     *  - Angle increases as GyroSystem is turned clockwise as seen from the top
     * 
     *  - "The angle is continuous, that is it will continue from 360 to 361 degrees. 
     *     This allows algorithms that wouldn't want to see a discontinuity in the GyroSystem 
     *     output as it sweeps past from 360 to 0 on the second time around."
     *      
     */
    public double getAngle() {
        return data.getAngle();
    }
    
    /**
     * @return double the rotation rate in degrees per second. Positive is clockwise.
     *  
     * Note:
     *  - Follows North-East-Down convention
     *      
     */
    @Override
    public double getTurnRate() {
        return data.getTurnRate();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return data.getHeading();
    }

    public Pose2d getPose() { 
        return odometry.getPoseMeters();
    } 

    public DifferentialDriveOdometry getOdometry() {
        return odometry;
    }

    /**
     * Method gets the wheel speeds using the encoders get velocity methods.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getEncoderVelocityLeft(), getEncoderVelocityRight());
    }
   
    private void configMotors() {
        SupplyCurrentLimitConfiguration currentLimitConfiguration = new SupplyCurrentLimitConfiguration(
            true, kDriveTrain.CurrentLimit, kDriveTrain.TriggerThresholdCurrent, kDriveTrain.triggerThresholdTime);

        // Left Front Drive
        mot_leftFrontDrive.configFactoryDefault();
        mot_leftFrontDrive.setInverted(kDriveTrain.Clockwise);
        mot_leftFrontDrive.setSensorPhase(kDriveTrain.kSensorPhase);
        mot_leftFrontDrive.configSupplyCurrentLimit(currentLimitConfiguration);

        mot_leftFrontDrive.configSelectedFeedbackSensor(
            TalonFXFeedbackDevice.IntegratedSensor, kDriveTrain.kPIDLoopIdx, kDriveTrain.kTimeoutMs);

        MotorUtils.setGains(mot_leftFrontDrive, kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains);

        // mot_leftFrontDrive.setSensorPhase(true);

        // Left Rear Drive
        mot_leftRearDrive.configFactoryDefault();
        mot_leftRearDrive.follow(mot_leftFrontDrive);
        mot_leftRearDrive.setInverted(InvertType.FollowMaster);
        mot_leftRearDrive.setSensorPhase(kDriveTrain.kSensorPhase);
        mot_leftRearDrive.configSupplyCurrentLimit(currentLimitConfiguration);
        
        mot_leftRearDrive.configSelectedFeedbackSensor(
            TalonFXFeedbackDevice.IntegratedSensor, kDriveTrain.kPIDLoopIdx, kDriveTrain.kTimeoutMs);

        MotorUtils.lowerFollowerStatusPeriod(mot_leftRearDrive);
        MotorUtils.setGains(mot_leftRearDrive, kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains);

        // mot_leftRearDrive.setSensorPhase(true);

        // Right Front Drive
        mot_rightFrontDrive.configFactoryDefault();
        mot_rightFrontDrive.setInverted(kDriveTrain.CounterClockwise);
        mot_rightFrontDrive.setSensorPhase(kDriveTrain.kSensorPhase);
        mot_rightFrontDrive.configSupplyCurrentLimit(currentLimitConfiguration);

        mot_rightFrontDrive.configSelectedFeedbackSensor(
            TalonFXFeedbackDevice.IntegratedSensor, kDriveTrain.kPIDLoopIdx, kDriveTrain.kTimeoutMs);

        MotorUtils.setGains(mot_rightFrontDrive, kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains);

        // mot_rightFrontDrive.setSensorPhase(true);

        // Right Rear Drive
        mot_rightRearDrive.configFactoryDefault();
        mot_rightRearDrive.follow(mot_rightFrontDrive);
        mot_rightRearDrive.setInverted(InvertType.FollowMaster);
        mot_rightRearDrive.setSensorPhase(kDriveTrain.kSensorPhase);
        mot_rightRearDrive.configSupplyCurrentLimit(currentLimitConfiguration);

        mot_rightRearDrive.configSelectedFeedbackSensor(
            TalonFXFeedbackDevice.IntegratedSensor, kDriveTrain.kPIDLoopIdx, kDriveTrain.kTimeoutMs);

        MotorUtils.lowerFollowerStatusPeriod(mot_rightRearDrive);
        MotorUtils.setGains(mot_rightRearDrive, kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains);
    }

    private void setDriveState(DriveState state) {
        if (state != this.state) {
            switch (state) {
                case kForward:
                    setRampRate(kDriveTrain.forwardRampRate);
                    break;
                case kForwardTurn:
                    setRampRate(kDriveTrain.forwardTurnRampRate);
                    break;
                case kReverse:
                    setRampRate(kDriveTrain.backwardRampRate);
                    break;
                case kReverseTurn:
                    setRampRate(kDriveTrain.backwardTurnRampRate);
                    break;
            }

            this.state = state;
        }
    }
}
