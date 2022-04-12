// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.DriveGear;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class SlowGear extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveTrain sys_drive;
  private boolean hasShift = false;

  /**
   * Creates a new SlowGear.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SlowGear(DriveTrain subsystem) {
    sys_drive = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // SmartDashboard.putBoolean("LowGear", true);

    hasShift = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rpmLeft = Math.abs(sys_drive.getRPMLeft());
    double rpmRight = Math.abs(sys_drive.getRPMRight());

    if(Constants.kConfig.DEBUG){
      SmartDashboard.putNumber("SHIFT LRPM", rpmLeft);
      SmartDashboard.putNumber("SHIFT RRPM", rpmRight);
    }

    if (rpmLeft < Constants.kDriveTrain.MAX_RPM_FOR_LOW_GEAR
        && rpmRight < Constants.kDriveTrain.MAX_RPM_FOR_LOW_GEAR && !hasShift) {
      sys_drive.setGearShift(DriveGear.kLowGear);
      hasShift = true;
      if(Constants.kConfig.DEBUG) SmartDashboard.putBoolean("Slow Shift True", true);
    } else {
      if(Constants.kConfig.DEBUG) SmartDashboard.putBoolean("Slow Shift False", true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hasShift;
  }
}
