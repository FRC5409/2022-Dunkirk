package frc.robot.commands.joystick;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.base.Joystick;
import frc.robot.base.command.DebounceScheduleCommand;

public class JoystickRumble extends CommandBase {
    private final Set<Joystick> joysticks;
    private final RumbleType type;
    private final double value;

    public JoystickRumble(Joystick joystick, double value) {
        this.joysticks = new HashSet<>();
        this.value = value;
        this.type = null;

        addJoysticks(joystick);
    }

    public JoystickRumble(Joystick joystick, RumbleType type, double value) {
        this.joysticks = Set.of(joystick);
        this.value = value;
        this.type = type;

        addJoysticks(joystick);
    }

    public JoystickRumble(double value) {
        this.joysticks = new HashSet<>();
        this.value = value;
        this.type = null;
    }

    public JoystickRumble(RumbleType type, double value) {
        this.joysticks = new HashSet<>();
        this.value = value;
        this.type = type;
    }


    @Override
    public void initialize() {
        if (type == null) {
            for (Joystick joystick : joysticks) {
                joystick.setRumble(value);
            }
        } else {
            for (Joystick joystick : joysticks) {
                joystick.setRumble(type, value);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        for (Joystick joystick : joysticks) {
            joystick.stopRumble();
        }
    }

    public JoystickRumble addJoysticks(Joystick... joysticks) {
        Set<Joystick> ungrouped = Set.of(joysticks);

        this.joysticks.addAll(ungrouped);
        m_requirements.addAll(ungrouped);

        return this;
    }

    public DebounceScheduleCommand withDebounce(double timeout) {
        return new DebounceScheduleCommand(timeout, this);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
