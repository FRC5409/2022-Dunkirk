package frc.robot.base;

import edu.wpi.first.wpilibj2.command.Command;

public interface RobotConfiguration {
    default public void teleopPeriodic() {
        
    }

    /**
     * Use this to pass the teleop command to the main {@link Robot} class.
     *
     * @return the command to run in teleop
     */
    default Command getTeleopCommand() {
        return null;
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    default Command getAutonomousCommand() {
        return null;
    }
}
