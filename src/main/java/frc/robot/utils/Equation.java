package frc.robot.utils;

/**
 * A simple interface for the representation of
 * mathematical equations,such as polynomials 
 * and other single variable equations.
 * 
 * @author Keith Davies
 */
@FunctionalInterface
public interface Equation {
    /**
     * Calculates the output of the equation
     * given input {@code x}.
     * 
     * @param x The input variable.
     * 
     * @return  The calculated output value.
     */
    public double calculate(double x);
}