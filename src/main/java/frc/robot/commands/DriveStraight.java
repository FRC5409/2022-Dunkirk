package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class DriveStraight extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private DriveTrain sys_drive;
    double leftSpeed;
    double rightSpeed;
  
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.

    public DriveStraight(DriveTrain subsystem, double d, double e) {
      sys_drive = subsystem;
      leftSpeed = d;
      rightSpeed = e;

      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(sys_drive);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        sys_drive.tankDrive(leftSpeed, rightSpeed);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        sys_drive.tankDrive(0,0);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
  }