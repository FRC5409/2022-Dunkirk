// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ElevateTo extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    // private final XboxController m_joystick;
    private final Climber climber;
    private double toPos;
    private boolean userVelForEnd = false;

    boolean started = false;

    private int direction = Constants.kClimber.DIRECTION_STATIONARY;
    private final Timer timer = new Timer();

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ElevateTo(Climber subsystem, double endPos) {
        climber = subsystem;
        toPos = endPos;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ElevateTo(Climber subsystem) {
        climber = subsystem;
        toPos = -1;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (toPos < 0)
            toPos = climber.getSliderPosition();

        climber.unlockArm();

        timer.reset();
        timer.start();

        started = false;
        if (toPos > climber.getPosition()) {
            direction = Constants.kClimber.DIRECTION_EXTEND;
        } else if (toPos < climber.getPosition()) {
            direction = Constants.kClimber.DIRECTION_RETRACT;
        }
    }

    @Override
    public void execute() {
        if (timer.hasElapsed(0.2) || !climber.getLocked()) {
            climber.moveArm(toPos);

            if (!started)
                started = true;

            timer.stop();
        }

        if (Math.abs(climber.getRPM()) > 1.0)
            userVelForEnd = true;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        climber.disableMotors();

        if (direction == Constants.kClimber.DIRECTION_RETRACT)
            climber.lockArm();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (climber.getDirection() == Constants.kClimber.DIRECTION_EXTEND && climber.getPosition() >= toPos)
                || (climber.getDirection() == Constants.kClimber.DIRECTION_RETRACT && climber.getPosition() <= toPos)
                || (userVelForEnd && Math.abs(climber.getRPM()) < 1.0);
    }
}
