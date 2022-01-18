package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.kGyroSystem;

public class Pigeon extends SubsystemBase{
    
    public final WPI_PigeonIMU gyro_pigeon;

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


    public Pigeon(){
        gyro_pigeon = new WPI_PigeonIMU(kGyroSystem.CANPigeon);
        gyro_pigeon.reset();
        
    }

    /**
     * This method is called once per scheduler run and is used to update smart dashboard data.
     */
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {
        updateAll();
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
    private double getRoll(){
        return gyro_pigeon.getRoll();
    }
    
    /**
     * @return double the GyroSystem's yaw
     */
    private double getYaw(){
        return gyro_pigeon.getYaw();
    }

    /**
     * @return double the GyroSystem's pitch
     */
    private double getPitch(){
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
    private double getAngle(){
        return gyro_pigeon.getAngle();
    }
    
    /**
     * @return double the rotation rate in degrees per second. Positive is clockwise.
     *  
     * Note:
     *  - Follows North-East-Down convention
     *      
     */
    private double getRate(){
        return gyro_pigeon.getRate();
    }

    /**
     * Puts the Roll-Pitch-Yaw into SmartDashboard 
     * 
     */
    public void displayAngle(){
        SmartDashboard.putNumber("Roll",  roll);
        SmartDashboard.putNumber("Pitch", pitch);
        SmartDashboard.putNumber("Yaw",   yaw);
    }

    /**
     * Puts the angle and rate into SmartDashboard 
     * 
     */
    public void displayHeading(){
        SmartDashboard.putNumber("Angle",  heading);
        SmartDashboard.putNumber("Rate", turn_rate);
    }
}
