package frc.robot.utils;

import edu.wpi.first.wpilibj.Timer;

/**
 * Watches over an object and requires frequent
 * updates in order to stay valid.
 * 
 * <p> Watchdogs can be used to monitor objects that must 
 * be updated and checked frequently. For example, 
 * a motor controller may use a watchdog in order 
 * to ensure that the motor is updated frequently enough. </p> 
 * 
 * @author Keith Davies
 */
public final class Watchdog {
    private final double expiry_time;

    private       double timer,
                         time_last;

    /**
     * Constructs a Watchdog.
     * 
     * @param expiry_time The time it takes for the watchdog
     *                    to expire when not sufficiently fed.
     */
    public Watchdog(double expiry_time) {
        this.expiry_time = expiry_time;

        timer = 0;
        time_last = Timer.getFPGATimestamp();
    }

    /**
     * Updates the watchdog timer.
     */
    public void update() {
        timer = Timer.getFPGATimestamp()-time_last;
    } 

    /**
     * Feeds / resets the watchdog timer.
     */
    public void feed() {
        timer = 0;
        time_last = Timer.getFPGATimestamp();
    }

    /**
     * Checks if the watchdog timer has
     * expired.
     * 
     * @return Whether or not the watchdog has expired.
     */
    public boolean isExpired() {
        return timer > expiry_time;
    }
}