package frc.robot.utils;

/**
 * Outlines functionality for objects
 * that can be enabled and disabled.
 * 
 * @author Keith Davies
 */
public abstract interface Toggleable {
    /**
     * Checks whether a group of toggleables
     * are all enabled.
     * 
     * @param requirements The toggleables to test.
     * 
     * @return Whether or not all of the toggleables are enabled.
     */
    public static boolean isEnabled(Toggleable... requirements) {
        for (Toggleable requisite : requirements) {
            if (!requisite.isEnabled())
                return false;
        }

        return true;
    }

    /**
     * Enables the toggleable.
     */
    public void enable();

    /**
     * Disables the subsystem.
     */
    public void disable();

    /**
     * Checks whether the toggleable
     * is enabled or disabled.
     * 
     * @return The toggleable state.
     */
    public boolean isEnabled();
}