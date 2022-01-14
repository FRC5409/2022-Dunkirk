package frc.robot.subsystems;



import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.kGyro;

public class Gyro extends SubsystemBase{
    
    public final WPI_Pigeon2 gyro_pigeon;


    public Gyro(){
        gyro_pigeon = new WPI_Pigeon2(kGyro.CANPigeon);
        gyro_pigeon.reset();
        
    }

    /**
     * This method is called once per scheduler run and is used to update smart dashboard data.
     */
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {

    }

    /**
     * resets the gyro's heading 
     */
    public void reset(){
        gyro_pigeon.reset();
    }

    /**
     * @return double the gyro's roll
     */
    public double getRoll(){
        return gyro_pigeon.getRoll();
    }
    
    /**
     * @return double the gyro's yaw
     */
    public double getYaw(){
        return gyro_pigeon.getYaw();
    }

    /**
     * @return double the gyro's pitch
     */
    public double getPitch(){
        return gyro_pigeon.getPitch();
    }

    /**
     * @return double the heading in degrees. Positive is clockwise.
     *  
     * Note:
     *  - Follows North-East-Down convention
     * 
     *  - Angle increases as gyro is turned clockwise as seen from the top
     * 
     *  - "The angle is continuous, that is it will continue from 360 to 361 degrees. 
     *     This allows algorithms that wouldn't want to see a discontinuity in the gyro 
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
        SmartDashboard.putNumber("Roll",  getRoll());
        SmartDashboard.putNumber("Pitch", getPitch());
        SmartDashboard.putNumber("Yaw",   getYaw());
    }

    /**
     * Puts the angle and rate into SmartDashboard 
     * 
     */
    public void displayHeading(){
        SmartDashboard.putNumber("Angle",  getAngle());
        SmartDashboard.putNumber("Rate", getRate());
    }
}
