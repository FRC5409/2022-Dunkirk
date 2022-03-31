// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

/** An example command that uses an example subsystem. */
public class ElevateTo extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    // private final XboxController m_joystick;
    private final Climber climber;
    private double toPos;
    private boolean useVelForEnd = false;
    private boolean lockOnDescend = false;

    boolean started = false;
    private final Timer timer = new Timer();

    /**
     * Creates a new instance of 
     * @param subsystem
     * @param endPos
     * @param _lockOnDescend
     */
    public ElevateTo(Climber subsystem, double endPos, boolean _lockOnDescend) {
        climber = subsystem;
        toPos = endPos;
        // Use addRequirements() here to declare subsystem dependencies.
        lockOnDescend = _lockOnDescend;

        addRequirements(subsystem);
    }

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

        climber.setPrevMove(toPos);
    }

    @Override
    public void execute() {
        if (!useVelForEnd && Math.abs(climber.getRPM()) >= 1.0)
            useVelForEnd = true;

        if (started)
            return;

        if (timer.hasElapsed(0.2) || !climber.getLocked()) {
            climber.moveArm(toPos);
            timer.stop();

            started = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        climber.disableMotors();

        if (lockOnDescend)
            climber.lockArm();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (climber.getDirection() == Constants.kClimber.DIRECTION_EXTEND && climber.getPosition() >= toPos)
                || (climber.getDirection() == Constants.kClimber.DIRECTION_RETRACT && climber.getPosition() <= toPos)
                || (useVelForEnd && Math.abs(climber.getRPM()) < 1.0);
    }
}
