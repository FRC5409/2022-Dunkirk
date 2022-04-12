package frc.robot.base.drive;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.Vector3;

public class PigeonData {
    // Memoize data table readings
    private final double[] data1;
    private final short[] data2;
    
    private Rotation2d rotation;
    private Vector3 acceleration;

    private double heading;
    private double pitch;
    private double angle;
    private double rate;
    private double roll;
    private double yaw;


    public PigeonData() {
        data1 = new double[3];
        data2 = new short[3];

        reset();
    }

    public void update(WPI_Pigeon2 pigeon) {
        pigeon.getYawPitchRoll(data1);
        yaw   = data1[2];
        roll  = data1[0];
        pitch = data1[1];

        pigeon.getBiasedAccelerometer(data2);
        acceleration.x = data2[0];
        acceleration.y = data2[1];
        acceleration.z = data2[2];
        
        rotation = pigeon.getRotation2d();
        heading = rotation.getDegrees();
        rate = pigeon.getRate();
    }

    public void reset() {
        acceleration = new Vector3();
        rotation = new Rotation2d();
        heading = rotation.getDegrees();
        pitch = 0;
        roll = 0;
        yaw = 0;
    }
    
    /**
     * @return the pigeons's roll
     */
    public double getRoll() {
        return roll;
    }
    
    /**
     * @return the pigeons's yaw
     */
    public double getYaw() {
        return yaw;
    }

    /**
     * @return the pigeons's pitch
     */
    public double getPitch() {
        return pitch;
    }

    /**
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
     * @return the pigeons in degrees. Positive is clockwise.    
     */
    public double getAngle() {
        return angle;
    }
    
    /**
     * @return the pigeons rate in degrees per second. Positive is clockwise.
     *  
     * Note:
     *  - Follows North-East-Down convention
     *      
     */
    public double getTurnRate() {
        return rate;
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return heading;
    }

    public Rotation2d getRotation2d() {
        return rotation;
    }

    public Vector3 getAcceleration() {
        return new Vector3(acceleration);
    }
}
