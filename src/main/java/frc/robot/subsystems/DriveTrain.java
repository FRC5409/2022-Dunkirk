package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kPneumatics;
import frc.robot.utils.Convert;
import frc.robot.utils.MotorUtils;
import frc.robot.Constants.kID;
import frc.robot.Constants;
import frc.robot.Constants.kDriveTrain;
import frc.robot.utils.Convert;

public class DriveTrain extends SubsystemBase {

    private final WPI_TalonFX mot_leftFrontDrive;
    private final WPI_TalonFX mot_leftRearDrive;
    private final WPI_TalonFX mot_rightFrontDrive;
    private final WPI_TalonFX mot_rightRearDrive;

    private final Timer timer = new Timer();
    private final double refreshSeconds = 2.0;

    private double lmRPM = 0;
    private double rmRPM = 0;

    private int driveMode;

    public final DifferentialDrive m_drive;

    private final DoubleSolenoid dsl_gear;

    private boolean applyAntiTip;

    private String drive_state;

    public final WPI_PigeonIMU gyro_pigeon;

    public DifferentialDriveOdometry m_odometry;

    // The robot's RPY
    public double roll;
    public double pitch;
    public double yaw;

    // The robot's angular velocity
    public double turn_rate;

    // The robot's heading
    public double heading;

    // XYZ acceleration relative to the robot
    public double x_acceleration;
    public double y_acceleration;
    public double z_acceleration;

    public DriveTrain() {
        // Left Front Drive
        mot_leftFrontDrive = new WPI_TalonFX(kID.LeftFrontDrive);

        // Left Rear Drive
        mot_leftRearDrive = new WPI_TalonFX(kID.LeftRearDrive);

        // Right Front Drive
        mot_rightFrontDrive = new WPI_TalonFX(kID.RightFrontDrive);

        // Right Rear Drive
        mot_rightRearDrive = new WPI_TalonFX(kID.RightRearDrive);

        configMotors();

        m_drive = new DifferentialDrive(mot_leftFrontDrive, mot_rightFrontDrive);

        // dsl_gear = new DoubleSolenoid(0, PneumaticsModuleType.REVPH,
        // kDriveTrain.ForwardChannel, kDriveTrain.ReverseChannel);

        dsl_gear = new DoubleSolenoid(kID.PneumaticHub, PneumaticsModuleType.REVPH, kDriveTrain.ForwardChannel,
                kDriveTrain.ReverseChannel);

        driveMode = kDriveTrain.InitialDriveMode;

        applyAntiTip = kDriveTrain.startWithAntiTip;

        setBrakeMode(true);

        zeroEncoders();

        gyro_pigeon = new WPI_PigeonIMU(kID.Pigeon);
        gyro_pigeon.reset();
        
        if(Constants.kConfig.DEBUG){
            SmartDashboard.putBoolean("Manual Override Enabled", false);

            SmartDashboard.putNumber("manual roll", 0);
            SmartDashboard.putNumber("manual pitch", 0);
            SmartDashboard.putNumber("manual yaw", 0);
        }

        m_odometry = new DifferentialDriveOdometry(gyro_pigeon.getRotation2d());


        // tof_front = new TimeOfFlight(Constants.kID.TOF_CLIMBER);
        // tof_front.setRangingMode(RangingMode.Medium, 1000);
        // 6630
        timer.start();
    }

    private void configMotors() {
        // Left Front Drive
        mot_leftFrontDrive.configFactoryDefault();
        mot_leftFrontDrive.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                kDriveTrain.kPIDLoopIdx,
                kDriveTrain.kTimeoutMs);
        mot_leftFrontDrive.setSensorPhase(kDriveTrain.kSensorPhase);
        mot_leftFrontDrive.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,
                kDriveTrain.CurrentLimit,
                kDriveTrain.TriggerThresholdCurrent,
                kDriveTrain.triggerThresholdTime));
        mot_leftFrontDrive.setInverted(kDriveTrain.Clockwise);

        mot_leftFrontDrive.configNominalOutputForward(0, kDriveTrain.kTimeoutMs);
        mot_leftFrontDrive.configNominalOutputReverse(0, kDriveTrain.kTimeoutMs);
        mot_leftFrontDrive.configPeakOutputForward(1, kDriveTrain.kTimeoutMs);
        mot_leftFrontDrive.configPeakOutputReverse(-1, kDriveTrain.kTimeoutMs);
        mot_leftFrontDrive.configAllowableClosedloopError(0, kDriveTrain.kPIDLoopIdx, kDriveTrain.kTimeoutMs);

        mot_leftFrontDrive.config_kF(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kF, kDriveTrain.kTimeoutMs);
        mot_leftFrontDrive.config_kP(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kP, kDriveTrain.kTimeoutMs);
        mot_leftFrontDrive.config_kI(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kI, kDriveTrain.kTimeoutMs);
        mot_leftFrontDrive.config_kD(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kD, kDriveTrain.kTimeoutMs);

        // Left Rear Drive
        mot_leftRearDrive.configFactoryDefault();
        mot_leftRearDrive.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                kDriveTrain.kPIDLoopIdx,
                kDriveTrain.kTimeoutMs);
        mot_leftRearDrive.setSensorPhase(kDriveTrain.kSensorPhase);

        mot_leftRearDrive.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,
                kDriveTrain.CurrentLimit,
                kDriveTrain.TriggerThresholdCurrent,
                kDriveTrain.triggerThresholdTime));
        mot_leftRearDrive.follow(mot_leftFrontDrive);
        mot_leftRearDrive.setInverted(InvertType.FollowMaster);
        MotorUtils.lowerFollowerStatusPeriod(mot_leftRearDrive);

        mot_leftRearDrive.configNominalOutputForward(0, kDriveTrain.kTimeoutMs);
        mot_leftRearDrive.configNominalOutputReverse(0, kDriveTrain.kTimeoutMs);
        mot_leftRearDrive.configPeakOutputForward(1, kDriveTrain.kTimeoutMs);
        mot_leftRearDrive.configPeakOutputReverse(-1, kDriveTrain.kTimeoutMs);
        mot_leftRearDrive.configAllowableClosedloopError(0, kDriveTrain.kPIDLoopIdx, kDriveTrain.kTimeoutMs);

        mot_leftRearDrive.config_kF(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kF, kDriveTrain.kTimeoutMs);
        mot_leftRearDrive.config_kP(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kP, kDriveTrain.kTimeoutMs);
        mot_leftRearDrive.config_kI(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kI, kDriveTrain.kTimeoutMs);
        mot_leftRearDrive.config_kD(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kD, kDriveTrain.kTimeoutMs);

        // Right Front Drive
        mot_rightFrontDrive.configFactoryDefault();
        mot_rightFrontDrive.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                kDriveTrain.kPIDLoopIdx,
                kDriveTrain.kTimeoutMs);
        mot_rightFrontDrive.setSensorPhase(kDriveTrain.kSensorPhase);
        mot_rightFrontDrive.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,
                kDriveTrain.CurrentLimit,
                kDriveTrain.TriggerThresholdCurrent,
                kDriveTrain.triggerThresholdTime));

        mot_rightFrontDrive.configNominalOutputForward(0, kDriveTrain.kTimeoutMs);
        mot_rightFrontDrive.configNominalOutputReverse(0, kDriveTrain.kTimeoutMs);
        mot_rightFrontDrive.configPeakOutputForward(1, kDriveTrain.kTimeoutMs);
        mot_rightFrontDrive.configPeakOutputReverse(-1, kDriveTrain.kTimeoutMs);
        mot_rightFrontDrive.configAllowableClosedloopError(0, kDriveTrain.kPIDLoopIdx, kDriveTrain.kTimeoutMs);

        mot_rightFrontDrive.config_kF(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kF, kDriveTrain.kTimeoutMs);
        mot_rightFrontDrive.config_kP(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kP, kDriveTrain.kTimeoutMs);
        mot_rightFrontDrive.config_kI(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kI, kDriveTrain.kTimeoutMs);
        mot_rightFrontDrive.config_kD(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kD, kDriveTrain.kTimeoutMs);
        mot_rightFrontDrive.setInverted(kDriveTrain.CounterClockwise);

        // Right Rear Drive
        mot_rightRearDrive.configFactoryDefault();
        mot_rightRearDrive.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                kDriveTrain.kPIDLoopIdx,
                kDriveTrain.kTimeoutMs);
        mot_rightRearDrive.setSensorPhase(kDriveTrain.kSensorPhase);
        mot_rightRearDrive.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,
                kDriveTrain.CurrentLimit,
                kDriveTrain.TriggerThresholdCurrent,
                kDriveTrain.triggerThresholdTime));
        mot_rightRearDrive.follow(mot_rightFrontDrive);
        mot_rightRearDrive.setInverted(InvertType.FollowMaster);
        MotorUtils.lowerFollowerStatusPeriod(mot_rightRearDrive);

        mot_rightRearDrive.configNominalOutputForward(0, kDriveTrain.kTimeoutMs);
        mot_rightRearDrive.configNominalOutputReverse(0, kDriveTrain.kTimeoutMs);
        mot_rightRearDrive.configPeakOutputForward(1, kDriveTrain.kTimeoutMs);
        mot_rightRearDrive.configPeakOutputReverse(-1, kDriveTrain.kTimeoutMs);
        mot_rightRearDrive.configAllowableClosedloopError(0, kDriveTrain.kPIDLoopIdx, kDriveTrain.kTimeoutMs);

        mot_rightRearDrive.config_kF(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kF, kDriveTrain.kTimeoutMs);
        mot_rightRearDrive.config_kP(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kP, kDriveTrain.kTimeoutMs);
        mot_rightRearDrive.config_kI(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kI, kDriveTrain.kTimeoutMs);
        mot_rightRearDrive.config_kD(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kD, kDriveTrain.kTimeoutMs);

        driveMode = kDriveTrain.InitialDriveMode;

        applyAntiTip = kDriveTrain.startWithAntiTip;

        drive_state = "";

        setBrakeMode(true);

        zeroEncoders();

        // tof_front = new TimeOfFlight(Constants.kID.TOF_CLIMBER);
        // tof_front.setRangingMode(RangingMode.Medium, 1000);
        // 6630
    }

    /**
     * This method is called once per scheduler run and is used to update smart
     * dashboard data.
     */
    public void periodic() {
        if(Constants.kConfig.DEBUG){
            displayEncoder();
        }

        displayTemperatures();

        if (timer.hasElapsed(refreshSeconds)) {
            if (mot_leftFrontDrive.hasResetOccurred()) {
                configMotors();
            }

            timer.reset();
        }

        m_odometry.update(
            gyro_pigeon.getRotation2d(), getEncoderPositionLeft(), getEncoderPositionRight());

    }

    @Override
    public void simulationPeriodic() {
        // displayEncoder();
        // displayDriveMode();
        // updateAll();
        // displayAngle();
    }


    /**
     * sets the ramp rate
     * 
     * @param rampRate time to full throttle
     * 
     */
    public void setRampRate(double rampRate){
        mot_leftFrontDrive.configOpenloopRamp(rampRate, kDriveTrain.kTimeoutMs);
        mot_rightFrontDrive.configOpenloopRamp(rampRate, kDriveTrain.kTimeoutMs);
    }


    // ------------------------- Drive Modes ------------------------- //

    /**
     * @param acceleration the robot's forward speed
     * 
     * @param deceleration the robot's backward speed
     * 
     * @param turn         the robot's angular speed about the z axis
     * 
     */
    public void aadilDrive(final double acceleration, final double deceleration, final double turn) {
        double accelerate = (acceleration - deceleration);


        //double rampRateAdjustment = (dsl_gear.get() == DoubleSolenoid.Value.kForward ? kDriveTrain.highGearRampRate : 0);

        if(accelerate > 0 && turn == 0 && drive_state != "forward"){
            drive_state = "forward";
            setRampRate(kDriveTrain.forwardRampRate);
        }
        else if(accelerate < 0 && turn == 0 && drive_state != "backwards"){
            drive_state = "backwards";
            setRampRate(kDriveTrain.backwardRampRate);
        }
        if(accelerate > 0 && turn != 0 && drive_state != "forward_turn"){
            drive_state = "forward_turn";
            setRampRate(kDriveTrain.forwardTurnRampRate);
        }
        else if(accelerate < 0 && turn != 0 && drive_state != "backward_turn"){
            drive_state = "backward_turn";
            setRampRate(kDriveTrain.backwardTurnRampRate);
        }

        m_drive.arcadeDrive(accelerate, turn, true);
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
        m_drive.tankDrive(leftSpeed, rightSpeed);
    }

    /**
     * @return the current drive mode as an int
     * 
     */
    public int getDriveMode() {
        return driveMode;
    }

    /**
     * @return the current drive mode as a String
     * 
     */
    private String getDriveModeString() {
        switch (driveMode) {
            case kDriveTrain.AADIL_DRIVE:
                return "AADIL DRIVE";

            case kDriveTrain.TANK_DRIVE:
                return "TANK DRIVE";

            default:
                return "NONE";
        }
    }

    /**
     * Puts the current drive mode into SmartDashboard
     * 
     */
    public void displayDriveMode() {
        SmartDashboard.putString("Drive Mode", getDriveModeString());
    }

    /**
     * directly set the drive mode to the specified int
     * 
     */
    public void setDriveMode(int newDriveMode) {
        driveMode = newDriveMode;
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
        System.out.println("Cycling drive mode");
        switch (driveMode) {
            case kDriveTrain.AADIL_DRIVE:
                driveMode = kDriveTrain.TANK_DRIVE;
                break;

            case kDriveTrain.TANK_DRIVE:
                driveMode = kDriveTrain.AADIL_DRIVE;
                break;
        }
    }

    /**
     * @param enable if true sets all motors to brake, if false sets all motors to
     *               coast.
     * 
     */
    public void setBrakeMode(boolean enable) {
        if (enable) {
            mot_leftFrontDrive.setNeutralMode(NeutralMode.Brake);
            mot_leftRearDrive.setNeutralMode(NeutralMode.Brake);
            mot_rightFrontDrive.setNeutralMode(NeutralMode.Brake);
            mot_rightRearDrive.setNeutralMode(NeutralMode.Brake);
        } else {
            mot_leftFrontDrive.setNeutralMode(NeutralMode.Coast);
            mot_leftRearDrive.setNeutralMode(NeutralMode.Coast);
            mot_rightFrontDrive.setNeutralMode(NeutralMode.Coast);
            mot_rightRearDrive.setNeutralMode(NeutralMode.Coast);
        }

    }

    // -------------------------- Temperature --------------------------- //

    void displayTemperatures() {
        SmartDashboard.putNumber("Temp LF", mot_leftFrontDrive.getTemperature());
        SmartDashboard.putNumber("Temp LB", mot_leftRearDrive.getTemperature());
        SmartDashboard.putNumber("Temp RF", mot_rightFrontDrive.getTemperature());
        SmartDashboard.putNumber("Temp RB", mot_rightRearDrive.getTemperature());
    }

    // ---------------------------- Anti Tip ---------------------------- //

    /**
     * @return true if the anti tip should be applied
     * 
     */
    public boolean getAntiTip() {
        return applyAntiTip;
    }

    /**
     * @param boolean the new state of applyAntiTip
     * 
     *                Sets applyAntiTip
     * 
     */
    public void setAntiTip(boolean _applyAntiTip) {
        applyAntiTip = !_applyAntiTip;
    }

    /**
     * toggles the anti-tip
     * 
     */
    public void toggleAntiTip() {
        applyAntiTip = !applyAntiTip;
    }

    // ---------------------------- Encoders ---------------------------- //

    /**
     * Puts the positions and velocities of the left and right encoders into
     * SmartDashboard
     * 
     */
    public void displayEncoder() {
        if (Math.abs(getRPMRight()) > rmRPM)
            rmRPM = Math.abs(getRPMRight());

        if (Math.abs(getRPMLeft()) > lmRPM)
            lmRPM = Math.abs(getRPMLeft());

        SmartDashboard.putNumber("Left Position", getEncoderPositionLeft());
        SmartDashboard.putNumber("Left Velocity", getEncoderVelocityLeft());
        SmartDashboard.putNumber("Left RPM", getRPMLeft());
        SmartDashboard.putNumber("Left MAX RPM", lmRPM);

        SmartDashboard.putNumber("Right Position", getEncoderPositionRight());
        SmartDashboard.putNumber("Right Velocity", getEncoderVelocityRight());
        SmartDashboard.putNumber("Right RPM", getRPMRight());
        SmartDashboard.putNumber("Right MAX RPM", rmRPM);
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
        double position = Convert.EncoderUnitsToInches((float)(mot_leftFrontDrive.getSelectedSensorPosition()+mot_leftRearDrive.getSelectedSensorPosition())/2);
        return Units.inchesToMeters(position);
    }

    /**
     * @return the average position of the right encoders
     * 
     */
    public double getEncoderPositionRight() {
        double position = Convert.EncoderUnitsToInches((float)(mot_rightFrontDrive.getSelectedSensorPosition()+mot_rightRearDrive.getSelectedSensorPosition())/2);
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
        double velocity = 10*Convert.EncoderUnitsToInches((float)(mot_leftFrontDrive.getSelectedSensorVelocity()+mot_leftRearDrive.getSelectedSensorVelocity())/2);
        return Units.inchesToMeters(velocity);
    }

    /**
     * @return the average velocity of the right encoders
     * 
     */
    public double getEncoderVelocityRight() {
        double velocity = 10*Convert.EncoderUnitsToInches((float)(mot_rightFrontDrive.getSelectedSensorVelocity()+mot_rightRearDrive.getSelectedSensorVelocity())/2);
        return Units.inchesToMeters(velocity);
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
     * Sets all encoders to 0
     * 
     * call this function when reading positions relative to the robot's current
     * position.
     * 
     */
    public void zeroEncoders() {
        mot_rightFrontDrive.setSelectedSensorPosition(0);
        mot_rightRearDrive.setSelectedSensorPosition(0);
        mot_leftFrontDrive.setSelectedSensorPosition(0);
        mot_leftRearDrive.setSelectedSensorPosition(0);
    }

    public void setAllEncoders(double position) {
        mot_rightFrontDrive.setSelectedSensorPosition(position);
        mot_rightRearDrive.setSelectedSensorPosition(position);
        mot_leftFrontDrive.setSelectedSensorPosition(position);
        mot_leftRearDrive.setSelectedSensorPosition(position);
    }

    public void setEncodersSplit(double position_left, double position_right){
        mot_rightFrontDrive.setSelectedSensorPosition(position_right);
        mot_rightRearDrive.setSelectedSensorPosition(position_right);
        mot_leftFrontDrive.setSelectedSensorPosition(position_left);
        mot_leftRearDrive.setSelectedSensorPosition(position_left);
    }

    // ------------------------ Setpoint Controls ------------------------ //

    public void setControlMode(TalonFXControlMode mode, double value) {
        mot_leftFrontDrive.set(mode, value);
        mot_leftRearDrive.set(mode, value);
        mot_rightFrontDrive.set(mode, value);
        mot_rightRearDrive.set(mode, value);
    }

    public void setControlMode(TalonFXControlMode mode, double value_l, double value_r) {
        mot_leftFrontDrive.set(mode, value_l);
        mot_leftRearDrive.set(mode, value_l);
        mot_rightFrontDrive.set(mode, value_r);
        mot_rightRearDrive.set(mode, value_r);
    }

    // ---------------------------- Solenoids ---------------------------- //

    /**
     * shifts the gear shift to fast
     */
    public void fastShift() {
        if(Constants.kConfig.DEBUG){
            SmartDashboard.putString("Solenoid", "Fast");
        }
        dsl_gear.set(DoubleSolenoid.Value.kForward);

    }

    /**
     * shifts the gear shift to slow
     */
    public void slowShift() {
        if(Constants.kConfig.DEBUG){
            SmartDashboard.putString("Solenoid", "Slow");
        }
        dsl_gear.set(DoubleSolenoid.Value.kReverse);
    }

    // ---------------------------- Pigeon ---------------------------- //

    // getters
    public double Roll(){
        return SmartDashboard.getBoolean("Manual Override Enabled", false) ? SmartDashboard.getNumber("manual roll", 0) : roll;
    }

    public double Pitch(){
        return SmartDashboard.getBoolean("Manual Override Enabled", false) ? SmartDashboard.getNumber("manual pitch", 0) : pitch;
    }

    public double Yaw(){
        return SmartDashboard.getBoolean("Manual Override Enabled", false) ? SmartDashboard.getNumber("manual yaw", 0) : yaw;
    }

    public double Heading(){
        return SmartDashboard.getBoolean("Manual Override Enabled", false) ? SmartDashboard.getNumber("manual heading", 0) : heading;
    }

    public double TurnRate(){
        return SmartDashboard.getBoolean("Manual Override Enabled", false) ? SmartDashboard.getNumber("manual turn_rate", 0) : turn_rate;
    }

    public double X_Acelleration(){
        return SmartDashboard.getBoolean("Manual Override Enabled", false) ? SmartDashboard.getNumber("manual x_acceleration", 0) : x_acceleration;
    }

    public double Y_Acelleration(){
        return SmartDashboard.getBoolean("Manual Override Enabled", false) ? SmartDashboard.getNumber("manual y_acceleration", 0) : y_acceleration;
    }

    public double Z_Acelleration(){
        return SmartDashboard.getBoolean("Manual Override Enabled", false) ? SmartDashboard.getNumber("manual x_acceleration", 0) : x_acceleration;
    }

    private void updateAll(){
        updateRPY();
        updateAcceleration();
        updateTurnAngle();
    }

    /**
     * Updates the roll pitch and yaw members
     */
    private void updateRPY(){
        double [] rpy = new double[3];
        gyro_pigeon.getYawPitchRoll(rpy);
        roll  = rpy[0];
        pitch = rpy[1];
        yaw   = rpy[2];
    }

    /**
     * Updates the heading and turn rate
     */
    private void updateTurnAngle(){
        heading   = getAngle();
        turn_rate = getRate();
    }

    /**
     * Updates the xyz acceleration members
     */
    private void updateAcceleration(){
        short [] xyz = new short[3];
        gyro_pigeon.getBiasedAccelerometer(xyz);
        x_acceleration = xyz[0];
        y_acceleration = xyz[1];
        z_acceleration = xyz[2];
    }

    /**
     * resets the GyroSystem's heading 
     */
    public void resetGyro(){
        gyro_pigeon.reset();
    }

    /**
     * @return double the GyroSystem's roll
     */
    public double getRoll(){
        return gyro_pigeon.getRoll();
    }
    
    /**
     * @return double the GyroSystem's yaw
     */
    public double getYaw(){
        return gyro_pigeon.getYaw();
    }

    /**
     * @return double the GyroSystem's pitch
     */
    public double getPitch(){
        return gyro_pigeon.getPitch();
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
    public double getAngle(){
        return gyro_pigeon.getAngle();
    }
    
    /**
     * @return double the rotation rate in degrees per second. Positive is clockwise.
     *  
     * Note:
     *  - Follows North-East-Down convention
     *      
     */
    public double getRate(){
        return gyro_pigeon.getRate();
    }

    /**
     * Puts the Roll-Pitch-Yaw into SmartDashboard 
     * 
     */
    public void displayAngle(){
        SmartDashboard.putNumber("Roll",  Roll());
        SmartDashboard.putNumber("Pitch", Pitch());
        SmartDashboard.putNumber("Yaw",   Yaw());
    }

    /**
     * Puts the angle and rate into SmartDashboard 
     * 
     */
    public void displayHeading(){
        SmartDashboard.putNumber("Angle",  Heading());
        SmartDashboard.putNumber("Rate", TurnRate());
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return gyro_pigeon.getRotation2d().getDegrees();
    }

    // ---------------------------- Auto ---------------------------- //

    /**
     * Method gets the wheel speeds using the encoders get velocity methods.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getEncoderVelocityLeft(), getEncoderVelocityRight());
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
        m_drive.feed();
    }

    public Pose2d getPose(){
        return m_odometry.getPoseMeters();
    }

    public DifferentialDriveOdometry getOdometry(){
        return m_odometry;
    }

    public void resetOdometry(Pose2d pose) {
        zeroEncoders();
        m_odometry.resetPosition(pose, gyro_pigeon.getRotation2d());
    }
}
