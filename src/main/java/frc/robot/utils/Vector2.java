package frc.robot.utils;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * Simple 2D Cartesian plane
 * point / vector / coordinate.
 * 
 * @author Keith Davies
 */
public class Vector2 implements Sendable {
    /**
     * X Coordinate
     */
    public double x;
    
    /**
     * Y Coordinate
     */
    public double y;

    /**
     * Construct Blank {@link Vector2}.
     */
    public Vector2() {
        x = 0;
        y = 0;
    }

    /**
     * Construct {@link Vector2} with coordinates.
     * 
     * @param x X Coordinate
     * @param y Y Coordinate
     */
    public Vector2(double x, double y) {
        this.x = x;
        this.y = y;
    }


    /**
     * Construct {@link Vector2} from another {@link Vector2}.
     * 
     * @param other A {@link Vector2}.
     */
    public Vector2(Vector2 other) {
        this.x = other.x;
        this.y = other.y;
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
    public Vector2 unit() {
        final double m = magnitude();
        if (m == 0) return new Vector2();
        return new Vector2(x / m, y / m);
    }

    /**
     * Scale vector by factor.
     * 
     * @param magnitude The scale factor.
     * 
     * @return A scaled vector
     */
    public Vector2 scale(double factor) {
        return new Vector2(x * factor, y * factor);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("x", () -> x, _x -> x = _x);
        builder.addDoubleProperty("y", () -> y, _y -> y = _y);
    }

    @Override
    public boolean equals(Object o) {
        if (!(o instanceof Vector2)) return false;
        Vector2 other = (Vector2) o;
        return x == other.x && y == other.y;
    }

    @Override
    public String toString() {
        return "(" + x + ", " + y + ")";
    }
}
