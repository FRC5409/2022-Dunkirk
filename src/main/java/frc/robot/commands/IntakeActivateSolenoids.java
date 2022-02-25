package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;


import frc.robot.subsystems.Intake;

   
public class IntakeActivateSolenoids extends CommandBase{
   
private final Intake sys_Intake;

  public IntakeActivateSolenoids(Intake subsystem) {
    
    sys_Intake = subsystem; 

    addRequirements(subsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    if(sys_Intake.isExtended()){
        sys_Intake.solenoidsUp();
    } else {
        sys_Intake.solenoidsDown();
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }

}

