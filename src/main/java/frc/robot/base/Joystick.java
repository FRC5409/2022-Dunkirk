package frc.robot.base;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class Joystick implements Subsystem {
    public static enum ButtonType {
        kLeftBumper,
        kRightBumper,
        kLeftStick,
        kRightStick,
        kA,
        kB,
        kX,
        kY,
        kBack,
        kStart,
        kLeftPov,
        kRightPov,
        kUpPov,
        kDownPov
    }

    private final XboxController m_controller;
    private final JoystickButton m_buttonA;
    private final JoystickButton m_buttonB;
    private final JoystickButton m_buttonX;
    private final JoystickButton m_buttonY;
    private final JoystickButton m_buttonLeftBumper;
    private final JoystickButton m_buttonRightBumper;
    private final JoystickButton m_buttonLeftStick;
    private final JoystickButton m_buttonRightStick;
    private final JoystickButton m_buttonBack;
    private final JoystickButton m_buttonStart;
    private final Button         m_buttonPovUp;
    private final Button         m_buttonPovDown;
    private final Button         m_buttonPovLeft;
    private final Button         m_buttonPovRight;

    public Joystick(int port) {
        m_controller = new XboxController(port);

        m_buttonA           = new JoystickButton(m_controller, XboxController.Button.kA.value);
        m_buttonB           = new JoystickButton(m_controller, XboxController.Button.kB.value);
        m_buttonX           = new JoystickButton(m_controller, XboxController.Button.kX.value);
        m_buttonY           = new JoystickButton(m_controller, XboxController.Button.kY.value);
        m_buttonLeftBumper  = new JoystickButton(m_controller, XboxController.Button.kLeftBumper.value);
        m_buttonRightBumper = new JoystickButton(m_controller, XboxController.Button.kRightBumper.value);
        m_buttonLeftStick   = new JoystickButton(m_controller, XboxController.Button.kLeftStick.value);
        m_buttonRightStick  = new JoystickButton(m_controller, XboxController.Button.kRightStick.value);
        m_buttonBack        = new JoystickButton(m_controller, XboxController.Button.kBack.value);
        m_buttonStart       = new JoystickButton(m_controller, XboxController.Button.kStart.value);

        m_buttonPovUp = new Button(
            () -> {
                int i = m_controller.getPOV();
                return (i == 0) || (i == 45) || (i == 315);
            }
        );

        m_buttonPovRight = new Button(
            () -> {
                int i = m_controller.getPOV();
                return (i == 45) || (i == 90) || (i == 135);
            }
        );
        
        m_buttonPovDown = new Button(
            () -> {
                int i = m_controller.getPOV();
                return (i == 135) || (i == 180) || (i == 225);
            }
        );
        
        m_buttonPovLeft = new Button(
            () -> {
                int i = m_controller.getPOV();
                return (i == 225) || (i == 270) || (i == 315);
            }
        );
    }

    public void setRumble(double value) {
        m_controller.setRumble(RumbleType.kLeftRumble, value);
        m_controller.setRumble(RumbleType.kRightRumble, value);
    }

    public void setRumble(RumbleType type, double value) {
        m_controller.setRumble(type, value);
    }

    public void stopRumble(RumbleType type) {
        m_controller.setRumble(type, 0);
    }

    public void stopRumble() {
        m_controller.setRumble(RumbleType.kLeftRumble, 0);
        m_controller.setRumble(RumbleType.kRightRumble, 0);
    }

    public XboxController getController() {
        return m_controller;
    }

    public Button getButton(ButtonType type) {
        switch (type) {
            case kLeftBumper:  return m_buttonLeftBumper;
            case kRightBumper: return m_buttonRightBumper;
            case kLeftStick:   return m_buttonLeftStick;
            case kRightStick:  return m_buttonRightStick;
            case kA:           return m_buttonA;
            case kB:           return m_buttonB;
            case kX:           return m_buttonX;
            case kY:           return m_buttonY;
            case kBack:        return m_buttonBack;
            case kStart:       return m_buttonStart;
            case kLeftPov:     return m_buttonPovLeft;
            case kRightPov:    return m_buttonPovRight;
            case kUpPov:       return m_buttonPovUp;
            case kDownPov:     return m_buttonPovDown;
            default: throw new IllegalArgumentException();        
        }
    }
}
