// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;

import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class IndexerIntakeActive extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private Indexer sys_indexer;
  private Intake sys_intake;

  boolean TOF_Ent = false;
  boolean TOF_Ball1 = false;
  boolean TOF_Ext = false;

  String state = "";

  /**
   * Creates a new ExampleCommand.
   *
   * @param indexer The subsystem used by this command.
   */
  public IndexerIntakeActive(Indexer indexer, Intake intake) {
    sys_indexer = indexer;
    sys_intake = intake;

    

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexer, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    sys_intake.intakeOn(0.4);
    //sys_intake.intakeIn(1);
    sys_indexer.indexerOn(1);
    sys_intake.solenoidsDown();

    state = "running";
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    TOF_Ent = sys_indexer.ballDetectionEnter();
    TOF_Ball1 = sys_indexer.ballDetectionBall1();
    TOF_Ext = sys_indexer.ballDetectionExit();
    /*
    if(TOF_Ent){
      sys_indexer.indexerOn(0.75);
    } 
    else if(TOF_Ball1 && !TOF_Ext){
      sys_indexer.indexerOn(0);
    } 
    else if(TOF_Ext){
      sys_indexer.indexerOn(0);
    }
    */
    
    if(TOF_Ball1 && TOF_Ext){
      sys_indexer.indexerOn(0);
    } else{
      sys_indexer.indexerOn(0.75);
    }
    
    /*
    if(state == "default"){
      sys_indexer.indexerOn(0);
      System.out.println(state);

      // exit conditions
      if(TOF_Ball1 && !TOF_Ext){
        // running
        state = "running";
      }
      else if(!TOF_Ball1 && !TOF_Ext){
        // running
        state = "running";
      }
      else if(TOF_Ball1 && TOF_Ext){
        state = "holding";
      }
    }
    
    if(state == "running"){
      System.out.println(state);
      sys_indexer.indexerOn(0.75);

      if(TOF_Ball1 && TOF_Ext){
        state = "run_back";
      }
    }
    else if(state == "run_back"){
      System.out.println(state);
      sys_indexer.setControlMode(-2000, ControlType.kPosition);
      state = "holding";
    }
    else if(state == "holding"){
      System.out.println(sys_indexer.encoderPosition());
      if(Math.abs(sys_indexer.encoderPosition() - -2000d) < 0.05){
        System.out.println("Ended");
        sys_indexer.setControlMode(0, ControlType.kDutyCycle);
      }
      // what do i do after it ends
    }
    */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sys_indexer.indexerOn(0);
    sys_intake.intakeOn(0);
    sys_intake.intakeIn(0);
    sys_intake.solenoidsUp();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // if (TOF_Ball1 && TOF_Ext) {

    //   return true;
    // }

    return false;
  }
}
