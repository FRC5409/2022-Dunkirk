package frc.robot.base;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class Joystick {
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
        kStart
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

    public Joystick(int port) {
        m_controller = new XboxController(port);

        m_buttonA = new JoystickButton(m_controller, XboxController.Button.kA.value);
        m_buttonB = new JoystickButton(m_controller, XboxController.Button.kB.value);
        m_buttonX = new JoystickButton(m_controller, XboxController.Button.kX.value);
        m_buttonY = new JoystickButton(m_controller, XboxController.Button.kY.value);
        m_buttonLeftBumper = new JoystickButton(m_controller, XboxController.Button.kLeftBumper.value);
        m_buttonRightBumper = new JoystickButton(m_controller, XboxController.Button.kRightBumper.value);
        m_buttonLeftStick = new JoystickButton(m_controller, XboxController.Button.kLeftStick.value);
        m_buttonRightStick = new JoystickButton(m_controller, XboxController.Button.kRightStick.value);
        m_buttonBack = new JoystickButton(m_controller, XboxController.Button.kBack.value);
        m_buttonStart = new JoystickButton(m_controller, XboxController.Button.kStart.value);
    }

    public XboxController getController() {
        return m_controller;
    }

    public JoystickButton getButton(ButtonType type) {
        switch (type) {
            case kLeftBumper: return m_buttonLeftBumper;
            case kRightBumper: return m_buttonRightBumper;
            case kLeftStick: return m_buttonLeftStick;
            case kRightStick: return m_buttonRightStick;
            case kA: return m_buttonA;
            case kB: return m_buttonB;
            case kX: return m_buttonX;
            case kY: return m_buttonY;
            case kBack: return m_buttonBack;
            case kStart: return m_buttonStart;
            default: throw new IllegalArgumentException();        
        }
    }
}
