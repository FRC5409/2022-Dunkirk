// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode;
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

        UsbCamera camera_wide = CameraServer.startAutomaticCapture();
        int[] cw_defaultRes = { 1920, 1080 };
        camera_wide.setVideoMode(PixelFormat.kMJPEG, cw_defaultRes[0],
        cw_defaultRes[1], 30);
        camera_wide.setResolution(-cw_defaultRes[0], cw_defaultRes[1]);
        camera_wide.setFPS(15);

        String config = "{ \"properties \": [{ \"name \":  \"connect_verbose \", \"value \": 1},{ \"name \":  \"raw_brightness \", \"value \": 2},{ \"name \":  \"brightness \", \"value \": 52},{ \"name \":  \"raw_contrast \", \"value \": 32},{ \"name \":  \"contrast \", \"value \": 50},{ \"name \":  \"raw_saturation \", \"value \": 64},{ \"name \":  \"saturation \", \"value \": 49},{ \"name \":  \"raw_hue \", \"value \": 0},{ \"name \":  \"hue \", \"value \": 50},{ \"name \":  \"white_balance_temperature_auto \", \"value \": true},{ \"name \":  \"gamma \", \"value \": 79},{ \"name \":  \"raw_gain \", \"value \": 0},{ \"name \":  \"gain \", \"value \": 0},{ \"name \":  \"power_line_frequency \", \"value \": 1},{ \"name \":  \"white_balance_temperature \", \"value \": 4600},{ \"name \":  \"raw_sharpness \", \"value \": 5},{ \"name \":  \"sharpness \", \"value \": 83},{ \"name \":  \"backlight_compensation \", \"value \": 1},{ \"name \":  \"exposure_auto \", \"value \": 1},{ \"name \":  \"raw_exposure_absolute \", \"value \": 625},{ \"name \":  \"exposure_absolute \", \"value \": 12},{ \"name \":  \"exposure_auto_priority \", \"value \": true}], \"width \": -1920}";
        camera_wide.setConfigJson(config);
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
