// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class IntakeIndexGo extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private Intake sys_intake;
  private Indexer sys_indexer;

  int countBalls; 
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntakeIndexGo(Indexer indexer, Intake intake) {
    sys_intake = intake;
    sys_indexer = indexer;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sys_intake);
    addRequirements(sys_indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if(countBalls == 2){
      sys_intake.intakeOn(1);
      sys_intake.solenoidsDown();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sys_intake.intakeOn(0);
    sys_indexer.indexerOn(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
