// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class MoveClimberArm extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    // private final XboxController m_joystick;
    private final Climber climber;
    private double toPos = 0;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public MoveClimberArm(Climber subsystem) {
        climber = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        toPos = climber.getSliderPosition();

        System.out.println("Going to " + toPos + "m.");
        toPos /= (Constants.Climber.CIRCUMFERENCE / Constants.Climber.GEAR_RATIO);

        climber.moveArm(toPos);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        climber.disableMotors();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return climber.getPosition() >= toPos;
    }
}
