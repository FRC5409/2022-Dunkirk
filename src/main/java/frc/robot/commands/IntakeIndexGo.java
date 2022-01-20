// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeIndexer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class IntakeIndexGo extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private IntakeIndexer sys_intakeIndexer;

  boolean indexerRun; 

  protected boolean m_triggered; 

  String m_colourSensor_etr;

  String m_colourSensor_ext; 

  String allianceColour; 

  int countBalls; 
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntakeIndexGo(IntakeIndexer subsystem) {
    sys_intakeIndexer = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_colourSensor_etr = "nicole method for colour"; 
    m_colourSensor_ext = "nicole method for colour"; 
    allianceColour = "nicole method for fms"; 

    if(allianceColour == "B"){ // alliance colour is blue
        if(m_colourSensor_etr == allianceColour && m_colourSensor_ext == "U"){
          countBalls ++; 
          //moveIndexer()
        } else if(m_colourSensor_ext == allianceColour && m_colourSensor_etr == allianceColour){
          //stop indexer
          countBalls ++;
        }
    } else if(allianceColour == "R"){ //alliance colour is red
      if(m_colourSensor_etr == allianceColour && m_colourSensor_ext == "U"){
        countBalls ++; 
        //move indexer
      } else if(m_colourSensor_ext == allianceColour && m_colourSensor_etr == allianceColour){
        //stop indexer
        countBalls ++;
      }
    } 


    
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
