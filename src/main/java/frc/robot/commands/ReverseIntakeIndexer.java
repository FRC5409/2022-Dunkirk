// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeIndexer;

public class ReverseIntakeIndexer extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private IntakeIndexer sys_intakeIndexer;

    char m_colourSensor_etr;
    char m_colourSensor_ext;
    char allianceColour; 

    public ReverseIntakeIndexer(IntakeIndexer subsystem) {
        sys_intakeIndexer = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
      }
    
      // Called when the command is initially scheduled
      @Override
      public void initialize(){}

      // Called everytime the scheduler runs while the command is scheduled
      @Override
      public void execute() {
          m_colourSensor_etr = sys_intakeIndexer.getEntranceColour();
          m_colourSensor_ext = sys_intakeIndexer.getExitColour();
          allianceColour = sys_intakeIndexer.getFMS();

          if(allianceColour != m_colourSensor_etr){
            sys_intakeIndexer.reverseIndexer(1);
            sys_intakeIndexer.reverseIntake(1);
          } else if(allianceColour != m_colourSensor_ext){
              //flop shoot
          }

      }

      // Called once the command ends or is interuppted
      @Override
      public void end(boolean interuppted){}

      // Returns true when when the command should end
      @Override
      public boolean isFinished(){
          return false;
      }


    
}
