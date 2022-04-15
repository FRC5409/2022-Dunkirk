// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoException;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.lib.wpilib.TimedRobot;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private RobotContainer m_robotContainer;
    private Command m_autonomousCommand;
    private Command m_teleopCommand;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();
        UsbCamera cameraWide = CameraServer.startAutomaticCapture();
        // String config = cameraWide.getConfigJson();

        // String config = "{\"properties\": [{\"name\":\"width\",\"value\":100},
        // {\"name\": \"raw_Brightness\",\"value\": 0},{\"name\":
        // \"Brightness\",\"value\": 50},{\"name\": \"raw_Contrast\",\"value\":
        // 50},{\"name\": \"Contrast\",\"value\": 50},{\"name\": \"raw_Hue\",\"value\":
        // 0},{\"name\": \"Hue\",\"value\": 50},{\"name\": \"raw_Saturation\",\"value\":
        // 64},{\"name\": \"Saturation\",\"value\": 64},{\"name\":
        // \"raw_Sharpness\",\"value\": 50},{\"name\": \"Sharpness\",\"value\":
        // 50},{\"name\": \"Gamma\",\"value\": 300},{\"name\":
        // \"WhiteBalance\",\"value\": 4600},{\"name\":
        // \"BacklightCompensation\",\"value\": 0},{\"name\":
        // \"raw_Exposure\",\"value\": -6},{\"name\": \"Exposure\",\"value\": 25}]}";
        // cameraWide.setConfigJson(config);

        int[] cw_defaultRes = { 2048, 1536 };
        int cw_width = (int)(cw_defaultRes[0] / 2);
        int cw_height = (int)(cw_defaultRes[1] / 2);

        // System.out.println(cameraWide.getVideoMode().width);
        // System.out.println(cameraWide.getVideoMode().height);

        try {
            cameraWide.setVideoMode(cameraWide.getVideoMode().pixelFormat, cw_width, cw_height, 15);
            cameraWide.setFPS(15);
            cameraWide.setResolution(cw_width, cw_height);
            String config = cameraWide.getConfigJson();
            System.out.println(config);
            // System.out.println(cameraWide.getVideoMode().width);
            // System.out.println(cameraWide.getVideoMode().height);
            // cameraWide.setResolution(1, 1);
        } catch (VideoException ve) {
            ve.printStackTrace();
        }
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and
     * test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        m_teleopCommand = m_robotContainer.getTeleopCommand();

        if (m_teleopCommand != null) {
            m_teleopCommand.schedule();
        }
    }

    @Override
    public void teleopExit() {
        // TODO Auto-generated method stub
        super.teleopExit();
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        m_robotContainer.teleopPeriodic();
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }
}
