package frc.robot.utils;

/**
 * Simple 4D Cartesian plane
 * point / vector / coordinate.
 * 
 * @author Keith Davies
 */
public class Vector4 {
    /**
     * Construct Blank {@link Vector4}.
     */
    public Vector4() {
        x = 0;
        y = 0;
        z = 0;
        w = 0;
    }

    /**
     * Construct {@link Vector4} with coordinates.
     * 
     * @param x X Coordinate
     * @param y Y Coordinate
     * @param z Z Coordinate
     * @param w W Coordinate
     */
    public Vector4(double x, double y, double z, double w) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
    }

    /**
     * Construct {@link Vector4} from another {@link Vector4}.
     * 
     * @param other A {@link Vector4}.
     */
    public Vector4(Vector4 other) {
        this.x = other.x;
        this.y = other.y;
        this.z = other.z;
        this.w = other.w;
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
    
    /**
     * W Coordinate
     */
    public double w;
}

