package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;


public class RunIndexerBack extends CommandBase{
    private final Intake sys_Intake;
	private final Indexer sys_indexer;

	/**
	 * Creates a new RunIndexerBack
	 * 
	 * Command to run the indexer
	 */
	public RunIndexerBack(Intake intake, Indexer indexer) {
		sys_Intake = intake;
		sys_indexer = indexer;
		
		addRequirements(sys_Intake, sys_indexer);
	}

    @Override
    public void execute(){
		if(sys_indexer.ballDetectionExit()){
			sys_indexer.indexerOn(-0.5);
		}
    }

    @Override 
    public void end(boolean inter){
		
        sys_indexer.indexerOn(0);
    }
	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}


}

