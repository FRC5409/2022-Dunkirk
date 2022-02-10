// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class ReverseIntakeIndexer extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private Intake sys_intake;

    public ReverseIntakeIndexer(Intake subsystem) {
        sys_intake = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(sys_intake);
      }
    
      // Called when the command is initially scheduled
      @Override
      public void initialize(){}

      // Called everytime the scheduler runs while the command is scheduled
      @Override
      public void execute() {


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
