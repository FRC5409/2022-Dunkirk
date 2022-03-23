package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer.SensorType;


public class RunIndexerBack extends CommandBase{
    private final Intake intake;
	private final Indexer indexer;

	/**
	 * Creates a new RunIndexerBack
	 * 
	 * Command to run the indexer
	 */
	public RunIndexerBack(Intake intake, Indexer indexer) {
		this.intake = intake;
		this.indexer = indexer;
		
		addRequirements(intake, indexer);
	}

	@Override
	public void initialize() {
		indexer.enable();
	}

    @Override
    public void execute() {
		if(indexer.getSensorState(SensorType.kExit))
			indexer.setSpeed(-0.5);
    }

    @Override 
    public void end(boolean interrupted) {
		indexer.disable();
    }

	@Override
	public boolean isFinished() {
		return false;
	}


}

