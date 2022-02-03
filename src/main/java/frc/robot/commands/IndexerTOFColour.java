package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeIndexer;

public class IndexerTOFColour extends CommandBase{
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private IntakeIndexer sys_intakeIndexer;

  char m_colourSensor_etr;

  char allianceColour; 

  int countBalls; 

  boolean TOF_Exit;

  public IndexerTOFColour(IntakeIndexer subsystem){
      sys_intakeIndexer = subsystem;
      addRequirements(subsystem);
  }

  @Override
  public void initialize(){
    if(countBalls == 2){
        sys_intakeIndexer.intakeOn(1);
        sys_intakeIndexer.solenoidsDown();
      }
  }

  @Override 
  public void execute(){
    m_colourSensor_etr = sys_intakeIndexer.getEntranceColour();
    allianceColour = sys_intakeIndexer.getFMS();

    TOF_Exit = sys_intakeIndexer.ballDetectionExit();

    if(allianceColour == 'B'){ //if alliance colour is blue
        if(m_colourSensor_etr == allianceColour && TOF_Exit == false){
            sys_intakeIndexer.indexerOn(1);
            sys_intakeIndexer.intakeOn(0);
            countBalls++;
        } else if(m_colourSensor_etr == allianceColour && TOF_Exit == true){
            sys_intakeIndexer.indexerOn(0);
            countBalls++;
        }
    } 

    if(m_colourSensor_etr != allianceColour && TOF_Exit == false) countBalls = 0; 

    System.out.println(countBalls);

  }

  @Override
  public void end(boolean interuppted){
    sys_intakeIndexer.intakeOn(0);
    sys_intakeIndexer.indexerOn(0);

    if(sys_intakeIndexer.getExitColour() == allianceColour){
      sys_intakeIndexer.solenoidsDown();
    }
  }

  @Override
  public boolean isFinished(){return false;}
}
