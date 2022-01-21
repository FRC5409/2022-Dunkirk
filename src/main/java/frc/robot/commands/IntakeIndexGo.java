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

  char m_colourSensor_etr;

  char m_colourSensor_ext; 

  char allianceColour; 

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
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_colourSensor_etr = sys_intakeIndexer.getEntranceColour(); 
    m_colourSensor_ext = sys_intakeIndexer.getExitColour(); 
    allianceColour = sys_intakeIndexer.getFMS(); 

    if(allianceColour == 'B'){ // alliance colour is blue
        if(m_colourSensor_etr == allianceColour && m_colourSensor_ext == 'U'){
          countBalls ++; 
          sys_intakeIndexer.indexerOn(1); 
          sys_intakeIndexer.intakeOn(0);
        } else if(m_colourSensor_ext == allianceColour && m_colourSensor_etr == allianceColour){
          sys_intakeIndexer.indexerOn(0); 
          countBalls ++;
        }
    } else if(allianceColour == 'R'){ //alliance colour is red
      if(m_colourSensor_etr == allianceColour && m_colourSensor_ext == 'U'){
        countBalls ++; 
        sys_intakeIndexer.indexerOn(1); 
        sys_intakeIndexer.intakeOn(0);
      } else if(m_colourSensor_ext == allianceColour && m_colourSensor_etr == allianceColour){
        sys_intakeIndexer.indexerOn(0); 
        countBalls ++;
      }
    }
    
    System.out.println(countBalls);
    
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
