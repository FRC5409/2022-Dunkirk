package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;

public class IndexerTOFColour extends CommandBase{
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private Indexer sys_indexer;
  private Intake sys_intake;

  int countBalls; 

  boolean TOF_Exit;

  public IndexerTOFColour(Indexer subsystem){
    sys_indexer = subsystem;
    addRequirements(sys_indexer);
  }

  @Override
  public void initialize(){
    if(countBalls == 2){
      sys_intake.intakeOn(1);
      sys_intake.solenoidsDown();
    }
  }

  @Override 
  public void execute(){
    
  }

  @Override
  public void end(boolean interuppted){
    sys_intake.intakeOn(0);
    sys_indexer.indexerOn(0);

  }

  @Override
  public boolean isFinished(){return false;}
}
