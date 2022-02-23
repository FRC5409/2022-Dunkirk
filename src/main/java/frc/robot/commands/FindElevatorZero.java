// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.kDriveTrain;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Pigeon;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class FindElevatorZero extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final Climber climber;

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
        climber.findZero();
    }

    // Called every time the scheduler runs while the command is scheduled.
    // @Override
    // public void execute() {
       
    // }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        climber.disableMotors();
        climber.zeroEncoder();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return climber.getLimitSwitch();
    }
}
