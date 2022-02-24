package frc.robot.utils;

/**
 * Simple 2D Cartesian plane
 * point / vector / coordinate.
 * 
 * @author Keith Davies
 */
public class Vector2 {
    /**
     * Construct Blank Vector2.
     */
    public Vector2() {
        x = 0;
        y = 0;
    }

    /**
     * Construct Vector2 with coordinates.
     * 
     * @param x X Coordinate
     * @param y Y Coordinate
     */
    public Vector2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Compute dot product of this vector with
     * another. (A * B)
     * 
     * @param B Another Vector
     * 
     * @return Dot product
     */
    public double dot(Vector2 B) {
        return (x*B.x + y*B.y);
    }

    /**
     * Compute magnitude of this vector.
     * |A|
     * 
     * @return Magnitude of this vector
     */
    public double magnitude() {
        return Math.sqrt(x*x + y*y);
    }

    /**
     * Returns normalization of this vector.
     * A'
     * 
     * @return Magnitude of this vector
     */
    public Vector2 normalize() {
        final double mag = magnitude();
        return new Vector2(x/mag, y/mag);
    }

    /**
     * X Coordinate
     */
    public double x;
    
    /**
     * Y Coordinate
     */
    public double y;
}
