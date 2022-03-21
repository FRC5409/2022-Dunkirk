package frc.robot.utils;

/**
 * Simple 3D Cartesian plane
 * point / vector / coordinate.
 * 
 * @author Keith Davies
 */
public class Vector3 {
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
     * Compute the unit vector.
     * 
     * @return The unit vector.
     */ 
    public Vector3 unit() {
        final double m = magnitude();
        return new Vector3(x / m, y / m, z / m);
    }

    /**
     * Compute the cross product.
     * 
     * @return The cross product.
     */ 
    public Vector3 cross(Vector3 other) {
        return new Vector3(
            other.z * y - other.y * z,
            other.x * z - other.z * x,
            other.y * x - other.x * y
        );
    }

    /**
     * Scale vector by factor.
     * 
     * @param magnitude The scale factor.
     * 
     * @return A scaled vector
     */
    public Vector3 scale(double factor) {
        return new Vector3(x * factor, y * factor, z * factor);
    }

    @Override
    public boolean equals(Object o) {
        if (!(o instanceof Vector3)) return false;
        Vector3 other = (Vector3) o;
        return x == other.x && y == other.y && z == other.z;
    }

    @Override
    public String toString() {
        return "(" + x + ", " + y + ", " + z + ")";
    }
}

