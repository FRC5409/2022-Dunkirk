// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** A simple drive straight command that can be used by the DriveTrain */
public class TurnToAngleGyro extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveTrain sys_drive;
  // private final Pigeon sys_pigeon;
  private final double angle;
  private double startAngle;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TurnToAngleGyro(DriveTrain driveTrain, double toAngle) {
    sys_drive = driveTrain;
    angle = toAngle;

    addRequirements(sys_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Turning to angle");
    startAngle = sys_drive.getAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double left = -(1 * Math.abs((angle - (sys_drive.getAngle() % 360)) / startAngle));
    double right = (1 * Math.abs((angle - (sys_drive.getAngle() % 360)) / startAngle));

    if (Math.abs(left) < 0.5)
      left = -0.5;

    if (Math.abs(right) < 0.5)
      right = 0.5;

    sys_drive.tankDrive((float) left, (float) right);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sys_drive.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(angle - (sys_drive.getAngle() % 360)) <= 2;
    // return true;
  }
}
