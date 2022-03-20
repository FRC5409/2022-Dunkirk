package frc.robot.commands.indexer;

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
	public RunIndexerBack(Intake subsystem, Indexer indexer) {
		sys_Intake = subsystem;
		sys_indexer = indexer;
		
		addRequirements(sys_Intake, sys_indexer);
	}

	@Override
	public void initialize() {
		sys_indexer.enable();
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
		sys_indexer.disable();
    }
	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}


}
