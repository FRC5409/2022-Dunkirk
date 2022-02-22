// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** A simple drive straight command that can be used by the DriveTrain */
public class DriveToMidRung extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveTrain sys_drive;
  private final Climber sys_climber;
  boolean distanceValid = false;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveToMidRung(Climber _climber, DriveTrain _driveTrain) {
    sys_climber = _climber;
    sys_drive = _driveTrain;

    addRequirements(sys_drive);

    System.out.println("Starting drive");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sys_climber.clearDistances();
    distanceValid = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance = sys_climber.getDistance();

    if (!distanceValid)
      sys_drive.tankDrive(-0.5f, -0.5f);

    if (sys_climber.getValidDistance() && distance <= Constants.kDriveTrain.DISTANCE_TO_MID_RUN_FROM_WALL) {
      sys_climber.addDistance(distance);

      distanceValid = true;
    } else {
      distanceValid = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sys_drive.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return sys_climber.getNumOfDistances() >= 32;
  }
}
