// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.kDriveTrain;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Pigeon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class FindElevatorZero extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final Climber climber;
    private boolean started = false;
    private Timer timer = new Timer();

    /**
     * Creates a new DefaultDrive
     * .
     *
     * @param subsystem The subsystem used by this command.
     * @param joystick  The input device used by this command.
     */
    public FindElevatorZero(Climber _climber) {
        climber = _climber;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("Started");

        climber.unlockArm();

        started = false;

        timer.reset();
        timer.start();

        if (climber.getLimitSwitch())
            started = true;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (timer.hasElapsed(0.2) && !started) {
            climber.findZero();

            if (climber.getRPM() > 0 || timer.hasElapsed(0.4))
                started = true;
        }

        if (started && !climber.getLimitSwitch())
            climber.findZero();
        else if (started)
            climber.disableMotors();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("Ended");
        climber.disableMotors();
        climber.zeroEncoder();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(climber.getRPM()) < 1 && started;
    }
}
