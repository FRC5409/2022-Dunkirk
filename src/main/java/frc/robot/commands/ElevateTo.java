// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.Constants.ClimberDestination;
import frc.robot.subsystems.Climber;

/** An example command that uses an example subsystem. */
public class ElevateTo extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    // private final XboxController m_joystick;
    private final XboxController joystick;
    private final Climber climber;
    private double toPos;
    private boolean useVelForEnd = false;
    private boolean lockOnDescend = false;
    private Constants.ClimberDestination destination;

    boolean started = false;
    private final Timer timer = new Timer();
    private boolean isPressed = true;
    private boolean overriden = false;

    /**
     * Creates a new instance of
     * 
     * @param subsystem
     * @param endPos
     * @param _lockOnDescend
     */
    public ElevateTo(XboxController _joystick, Climber subsystem, Constants.ClimberDestination _destination,
            boolean _lockOnDescend) {
        this(_joystick, subsystem, _destination);

        lockOnDescend = _lockOnDescend;
    }

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ElevateTo(XboxController _joystick, Climber subsystem, Constants.ClimberDestination _destination) {
        joystick = _joystick;
        climber = subsystem;
        destination = _destination;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.

    private double getPos() {
        if (destination == Constants.ClimberDestination.slider)
            return climber.getSliderPosition();
        else if (destination == Constants.ClimberDestination.lowRung)
            return Constants.kClimber.TO_LOW_RUNG;
        else if (destination == Constants.ClimberDestination.midRung)
            return Constants.kClimber.TO_MID_RUNG;
        else if (destination == Constants.ClimberDestination.lockLow)
            return Constants.kClimber.TO_MIN_LOW;
        else if (destination == Constants.ClimberDestination.lockMid)
            return Constants.kClimber.TO_MIN_MID;
        else
            return Constants.kClimber.TO_MIN_LOW;
    }

    @Override
    public void initialize() {

        climber.unlockArm();

        timer.reset();
        timer.start();

        started = false;
        overriden = false;

        toPos = getPos();
        climber.setPrevMove(toPos);

        if (lockOnDescend) {
            climber.lockArm();
        }
    }

    @Override
    public void execute() {
        int pov = joystick.getPOV();

        if (!isPressed) {
            if (pov == 0 && destination != Constants.ClimberDestination.midRung) { // Go to mid rung
                lockOnDescend = false;
                // CommandScheduler.getInstance().schedule(true, new ElevateTo(joystick,
                // climber, ClimberDestination.midRung));
                destination = ClimberDestination.midRung;
                toPos = getPos();

                climber.unlockArm();
                timer.reset();

                started = false;

                isPressed = true;

            } else if (pov == 180 && (destination != Constants.ClimberDestination.lockLow
                    || destination != Constants.ClimberDestination.lockMid)) { // Send elevator down to position based
                                                                               // on
                                                                               // previous held position
                // overriden = true;
                // CommandScheduler.getInstance().schedule(true,
                // new ElevateTo(joystick, climber,
                // (climber.getPrevMove() == Constants.kClimber.TO_LOW_RUNG)
                // ? ClimberDestination.lockLow
                // : ClimberDestination.lockMid,
                // true));
                System.out.println(destination);
                // destination = (destination == ClimberDestination.lowRung)
                //         ? ClimberDestination.lockLow
                //         : ClimberDestination.lockMid;
                destination = ClimberDestination.lockLow;
                toPos = getPos();

                climber.lockArm();
                climber.moveArm(toPos);
                isPressed = true;

            } else if (pov == 270 && destination != ClimberDestination.lowRung) { // Go to low rung
                overriden = true;

                // CommandScheduler.getInstance().schedule(true, new ElevateTo(joystick,
                // climber, ClimberDestination.lowRung));
                destination = ClimberDestination.lowRung;
                toPos = getPos();

                climber.unlockArm();
                timer.reset();

                started = false;
                isPressed = true;

            }
        } else if (pov == -1) {
            isPressed = false;
        }

        if (!useVelForEnd && Math.abs(climber.getRPM()) >= 1.0)
            useVelForEnd = true;

        if (started)
            return;

        if (destination == ClimberDestination.lockMid || destination == ClimberDestination.lockLow) {
            climber.moveArm(toPos);
            started = true;
        } else if (timer.hasElapsed(0.2) || !climber.getLocked()) {
            climber.moveArm(toPos);
            timer.stop();
            started = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        climber.disableMotors();
        climber.setPrevMove(toPos);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(climber.getPosition() - toPos) < 1.0;
    }
}
