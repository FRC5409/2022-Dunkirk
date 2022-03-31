// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/** An example command that uses an example subsystem. */
public class DefaultElevator extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final Climber climber;
    private final XboxController joystick;

    private final Timer timer = new Timer();

    /**
     * Creates a new DefaultElevator
     * .
     *
     * @param subsystem The subsystem used by this command.
     * @param joystick  The input device used by this command.
     */
    public DefaultElevator(Climber _climber, XboxController _joystick) {
        climber = _climber;
        joystick = _joystick;

        addRequirements(_climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        if (climber.getActive()) {
            climber.raiseFrames();
        } else {
            climber.lowerFrames();
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (climber.getActive()) {
            double l = joystick.getLeftTriggerAxis();
            double r = joystick.getRightTriggerAxis();

            if ((l > 0 || r > 0) && climber.getLocked()) {
                climber.unlockArm();
                timer.reset();
            }

            if (timer.hasElapsed(0.2) && !climber.getLocked()) {
                climber.moveArm(r, l);
                timer.stop();
            }

            int pov = joystick.getPOV();

            if (pov == 0) {
                CommandScheduler.getInstance().schedule(true, new ElevateTo(climber, Constants.kClimber.TO_MID_RUNG));
            } else if (pov == 90) {
                CommandScheduler.getInstance().schedule(false, new ToggleClimberLockCommand(climber));
            } else if (pov == 180) {
                CommandScheduler.getInstance().schedule(true,
                        new ElevateTo(climber,
                                (climber.getPrevMove() == Constants.kClimber.TO_LOW_RUNG)
                                        ? Constants.kClimber.TO_MIN_LOW
                                        : Constants.kClimber.TO_MIN_MID,
                                true));
            } else if (pov == 270) {
                CommandScheduler.getInstance().schedule(true, new ElevateTo(climber, Constants.kClimber.TO_LOW_RUNG));
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        timer.reset();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
