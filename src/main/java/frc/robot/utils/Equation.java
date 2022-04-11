package frc.robot.utils;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * A simple interface for the representation of
 * mathematical equations,such as polynomials 
 * and other single variable equations.
 * 
 * @author Keith Davies
 */
@FunctionalInterface
public interface Equation extends Sendable {
    /**
     * Calculates the output of the equation
     * given input {@code x}.
     * 
     * @param x The input variable.
     * 
     * @return  The calculated output value.
     */
    public double calculate(double x);

    @Override
    default void initSendable(SendableBuilder builder) {
        
    }
}