// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.kDriveTrain;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Pigeon;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/** An example command that uses an example subsystem. */
public class DefaultElevator extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final Climber climber;
    // private final Pigeon pigeon;
    private final XboxController joystick;

    /**
     * Creates a new DefaultDrive
     * .
     *
     * @param subsystem The subsystem used by this command.
     * @param joystick  The input device used by this command.
     */
    public DefaultElevator(Climber _climber, /* Pigeon _pigeon, */ XboxController _joystick) {
        climber = _climber;
        // pigeon = _pigeon;
        joystick = _joystick;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(_climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        climber.moveArm(joystick.getRightTriggerAxis(), joystick.getLeftTriggerAxis());

        int pov = joystick.getPOV();

        if (pov == 0) {
            CommandScheduler.getInstance().schedule(true, new ElevateTo(climber, Constants.kClimber.TO_MID_RUNG - 5));
        } else if (pov == 90) {
            CommandScheduler.getInstance().schedule(true, new ElevateTo(climber, Constants.kClimber.TO_MID_RUNG));
        } else if (pov == 180) {
            CommandScheduler.getInstance().schedule(true, new ElevateTo(climber, Constants.kClimber.TO_MIN));
        } else if (pov == 270)
            CommandScheduler.getInstance().schedule(true, new ElevateTo(climber, Constants.kClimber.TO_LOW_RUNG));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
