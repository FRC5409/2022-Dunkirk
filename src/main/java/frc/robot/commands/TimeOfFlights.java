// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeIndexer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class TimeOfFlights extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private IntakeIndexer sys_intakeIndexer;

  boolean TOF_Ent; 
  boolean TOF_Ext;

  int countBalls; 

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TimeOfFlights(IntakeIndexer subsystem) {
    sys_intakeIndexer = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sys_intakeIndexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    //safety to stop running the intake when indexer is full
    if(!(sys_intakeIndexer.ballDetectionExit() && sys_intakeIndexer.isRangeValid_Ext())){
      sys_intakeIndexer.intakeOn(1);
      sys_intakeIndexer.solenoidsDown();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    TOF_Ent = sys_intakeIndexer.ballDetectionEnter();
    TOF_Ext = sys_intakeIndexer.ballDetectionExit();

    if(TOF_Ent){
      sys_intakeIndexer.indexerOn(1);
      countBalls++;
    } else if(TOF_Ext){
      sys_intakeIndexer.indexerOn(0);
      countBalls++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sys_intakeIndexer.indexerOn(0);
    sys_intakeIndexer.intakeOn(0);

    if(sys_intakeIndexer.ballDetectionExit()){
      sys_intakeIndexer.solenoidsUp();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (sys_intakeIndexer.ballDetectionExit() && sys_intakeIndexer.isRangeValid_Ext());
  }
}
