// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.kDriveTrain;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DefaultDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain sys_drive;
  private final XboxController m_joystick;

  /**
   * Creates a new DefaultDrive
   *.
   *
   * @param subsystem The subsystem used by this command.
   * @param joystick The input device used by this command.
   */
  public DefaultDrive(DriveTrain subsystem, XboxController joystick) {
    sys_drive = subsystem;
    m_joystick = joystick;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      // Case to determine what control scheme to utilize 
      switch(sys_drive.getDriveMode()){
          case kDriveTrain.AADIL_DRIVE: //1
            aadilDriveExecute();
            break;
          case kDriveTrain.TANK_DRIVE: //2
            tankDriveExecute();
            break;
          default:
            aadilDriveExecute();
      }
      sys_drive.displayDriveMode();
  }

  /**
   * This method runs the robot with the aadilDrive control scheme
   */
  private void aadilDriveExecute(){
    double leftTrigger = m_joystick.getLeftTriggerAxis();
    double rightTrigger = m_joystick.getRightTriggerAxis();
    double lxAxis = m_joystick.getLeftX() * -1;

    sys_drive.aadilDrive(rightTrigger, leftTrigger, lxAxis);
  }

  /**
   * This method runs the robot with the tankDrive control scheme
   */
  private void tankDriveExecute(){
      double lyAxis = m_joystick.getLeftY() * -1; 
      double ryAxis = m_joystick.getRightY() * -1;

      sys_drive.tankDrive(lyAxis, ryAxis);
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
