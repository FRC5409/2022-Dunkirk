// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class IndexerActive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private Indexer sys_indexer;
  

  boolean TOF_Ent; 
  boolean TOF_Ball1; 
  boolean TOF_Ext;

  /**
   * Creates a new ExampleCommand.
   *
   * @param indexer The subsystem used by this command.
   */
  public IndexerActive(Indexer indexer) {
    sys_indexer = indexer;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    //safety to stop running the intake when indexer is full
    // if(!(sys_indexer.ballDetectionExit() && sys_indexer.isRangeValid_Ext())){
    //   sys_intake.intakeOn(0);
    //   sys_intake.solenoidsUp();
    // }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    TOF_Ent = sys_indexer.ballDetectionEnter();
    TOF_Ball1 = sys_indexer.ballDetectionBall1();
    TOF_Ext = sys_indexer.ballDetectionExit();

    if(TOF_Ent){
      sys_indexer.indexerOn(0.5);
    } else if(TOF_Ball1 && !TOF_Ext){
      sys_indexer.indexerOn(0);
    } else if(TOF_Ext){
      sys_indexer.indexerOn(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sys_indexer.indexerOn(0);
    // sys_intake.intakeOn(0);

    // if(sys_indexer.ballDetectionExit()){
    //   sys_intake.solenoidsUp();
    // }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    //(sys_indexer.ballDetectionExit() && sys_indexer.isRangeValid_Ext());
  }
}
