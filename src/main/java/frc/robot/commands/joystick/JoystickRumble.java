package frc.robot.commands.joystick;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.base.Joystick;

public class JoystickRumble extends CommandBase {
    private final RumbleType type;
    private final Joystick joystick;
    private final double value;

    public JoystickRumble(Joystick joystick, double value) {
        this.joystick = joystick;
        this.value = value;
        this.type = null;

        addRequirements(joystick);
    }

    public JoystickRumble(Joystick joystick, RumbleType type, double value) {
        this.joystick = joystick;
        this.value = value;
        this.type = type;
    }

    @Override
    public void initialize() {
        if (type == null)
            joystick.setRumble(value);
        else
            joystick.setRumble(type, value);
    }

    @Override
    public void end(boolean interrupted) {
        joystick.stopRumble();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
