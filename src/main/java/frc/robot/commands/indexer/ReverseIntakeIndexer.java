// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class ReverseIntakeIndexer extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private Intake sys_intake;
    private Indexer sys_indexer; 

    public ReverseIntakeIndexer(Intake intake, Indexer indexer) {
        sys_intake = intake;
        sys_indexer = indexer; 
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(intake, indexer);
      }
    
      // Called when the command is initially scheduled
      @Override
      public void initialize(){}

      // Called everytime the scheduler runs while the command is scheduled
      @Override
      public void execute() {
        sys_indexer.reverseIndexer(0.5);
        sys_intake.reverseIntake(0.75);
        // sys_intake.reverseIntakeIn(0.75);

      }

      // Called once the command ends or is interuppted
      @Override
      public void end(boolean interuppted){
          sys_intake.intakeOn(0);
          sys_indexer.indexerOn(0);
          // sys_intake.intakeIn(0);
      }

      // Returns true when when the command should end
      @Override
      public boolean isFinished(){
          return false;
      }
   
}
