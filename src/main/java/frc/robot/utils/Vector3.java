package frc.robot.utils;

/**
 * Simple 3D Cartesian plane
 * point / vector / coordinate.
 * 
 * @author Keith Davies
 */
public class Vector3 {
    /**
     * Construct Blank {@link Vector3}.
     */
    public Vector3() {
        x = 0;
        y = 0;
        z = 0;
    }

    /**
     * Construct {@link Vector3} with coordinates.
     * 
     * @param x X Coordinate
     * @param y Y Coordinate
     * @param z Z Coordinate
     */
    public Vector3(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    /**
     * Construct {@link Vector3} from another {@link Vector3}.
     * 
     * @param other A {@link Vector3}.
     */
    public Vector3(Vector3 other) {
        this.x = other.x;
        this.y = other.y;
        this.z = other.z;
    }

    /**
     * Compute magnitude of this vector.
     * |A|
     * 
     * @return Magnitude of this vector
     */
    public double magnitude() {
        return Math.sqrt(x*x + y*y + z*z);
    }

    /**
     * Compute normal of this vector.
     * 
     * @return Normal of this vector
     */ 
    public Vector3 norm() {
        final double mag = magnitude();
        return new Vector3(x / mag, y / mag, z / mag);
    }


    /**
     * X Coordinate
     */
    public double x;
    
    /**
     * Y Coordinate
     */
    public double y;
    
    /**
     * Z Coordinate
     */
    public double z;
}

