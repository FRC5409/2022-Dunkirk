// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.kDriveTrain;
import frc.robot.base.Property;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DefaultDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain drive;
  //private final Pigeon pigeon;
  private final XboxController joystick;

  private final Property<Double> driveSpeed;
  private double lastDriveSpeed;

  /**
   * Creates a new DefaultDrive
   *.
   *
   * @param subsystem The subsystem used by this command.
   * @param joystick The input device used by this command.
   */
  public DefaultDrive(DriveTrain _drive, /*Pigeon _pigeon,*/ XboxController _joystick, Property<Double> driveSpeed) {
    drive = _drive;
    //pigeon = _pigeon;
    joystick = _joystick;

    this.driveSpeed = driveSpeed;

    SmartDashboard.putNumber("Drive Speed Forward Smoothing", SmartDashboard.getNumber("Drive Speed Forward Smoothing", 0));
    SmartDashboard.putNumber("Drive Speed Reverse Smoothing", SmartDashboard.getNumber("Drive Speed Reverse Smoothing", 0));

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastDriveSpeed = driveSpeed.get();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      // Case to determine what control scheme to utilize 
      
      switch(drive.getDriveMode()){
          case kDriveTrain.AADIL_DRIVE: //1
            aadilDriveExecute();
            break;
          case kDriveTrain.TANK_DRIVE: //2
            tankDriveExecute();
            break;
          default:
            aadilDriveExecute();
      }
      if(Constants.kConfig.DEBUG) drive.displayDriveMode();
  }

  /**
   * This method runs the robot with the aadilDrive control scheme
   */
  private void aadilDriveExecute(){
    double lastDriveSpeed = driveSpeed.get();

    // double t = 0;
    // if (nextDriveSpeed - lastDriveSpeed > 0)
    //   t = SmartDashboard.getNumber("Drive Speed Forward Smoothing", 0);
    // else
    //   t = SmartDashboard.getNumber("Drive Speed Reverse Smoothing", 0);

    // lastDriveSpeed = MathUtil.interpolate(
    //   nextDriveSpeed, lastDriveSpeed, t
    // );

    SmartDashboard.putNumber("Drive Speed", lastDriveSpeed);

    double leftTrigger = joystick.getLeftTriggerAxis() * lastDriveSpeed;
    double rightTrigger = joystick.getRightTriggerAxis() * lastDriveSpeed;
    double lxAxis = joystick.getLeftX() * lastDriveSpeed;


    // Add when pigeon is online
    //double pitch = 90/pigeon.Pitch();
    //double roll  = 90/pigeon.Roll(); // might need to transform roll depending on where 0 is

    //double pitchCompensation = drive.getAntiTip() ? pitch*kDriveTrain.pitchCompensation : 0;
    //double rollCompensation = drive.getAntiTip() && (leftTrigger != 0) && (rightTrigger != 0)? roll * kDriveTrain.rollCompensation : 0;

    //drive.aadilDrive(Math.max(rightTrigger  - pitchCompensation - rollCompensation, 0), 
    //                 Math.max(leftTrigger + pitchCompensation + rollCompensation, 0), 
    //                 lxAxis);
    drive.aadilDrive(leftTrigger, rightTrigger, lxAxis);
    
  }

  private void debugDriveExecute(){
    double left = joystick.getLeftTriggerAxis() * (joystick.getLeftBumper() ? -1 : 1);
    double right = joystick.getRightTriggerAxis() * (joystick.getRightBumper() ? -1 : 1);

    drive.tankDrive(left, right);
  }

  /**
   * This method runs the robot with the tankDrive control scheme
   */
  private void tankDriveExecute(){
      double lyAxis = -joystick.getLeftY(); 
      double ryAxis = -joystick.getRightY();

      drive.tankDrive(lyAxis, ryAxis);
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
