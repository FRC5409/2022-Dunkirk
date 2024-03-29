package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.kDriveTrain;
import frc.robot.Constants.kID;

public class Pigeon extends SubsystemBase{
    
    public final WPI_Pigeon2 gyro_pigeon;
    private DifferentialDriveOdometry m_odometry;

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

    DriveTrain driveTrain = new DriveTrain();

    public Pigeon(){
        gyro_pigeon = new WPI_Pigeon2(kID.Pigeon);
        gyro_pigeon.reset();
        configPigeon();

        m_odometry = new DifferentialDriveOdometry(gyro_pigeon.getRotation2d());

        if(Constants.kConfig.DEBUG){
            SmartDashboard.putBoolean("Manual Override Enabled", false);
            SmartDashboard.putNumber("manual roll", 0);
            SmartDashboard.putNumber("manual pitch", 0);
            SmartDashboard.putNumber("manual yaw", 0);
        }

        m_odometry = new DifferentialDriveOdometry(gyro_pigeon.getRotation2d());

    }

    private void configPigeon(){
        gyro_pigeon.configMountPose(kDriveTrain.gyroMountYaw,
                                    kDriveTrain.gyroMountPitch, 
                                    kDriveTrain.gyroMountRoll  );
    }

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

    /**
     * This method is called once per scheduler run and is used to update smart dashboard data.
     */
    public void periodic() {
        m_odometry.update(
            gyro_pigeon.getRotation2d(), driveTrain.getEncoderPositionLeft(), driveTrain.getEncoderPositionRight()); 
        // TODO: causes drivetrain to move weirdly, also cause doiuble solenoid to return error
    }

    @Override
    public void simulationPeriodic() {
        updateAll();
        displayAngle();
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
    public void reset(){
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

    public Pose2d getPose(){
        return m_odometry.getPoseMeters();
    }

    public DifferentialDriveOdometry getOdometry(){
        return m_odometry;
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return gyro_pigeon.getRotation2d().getDegrees();
    }

    public void resetOdometry(Pose2d pose) {
        // System.out.println(pose);
        m_odometry.resetPosition(pose, gyro_pigeon.getRotation2d());
    }
}