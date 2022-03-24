package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;


public class IntakeActive extends CommandBase{
    private final Intake intake;
	private final Indexer indexer;

	/**
	 * Creates a new IntakeIndexActive
	 * 
	 * Command to run the indexer
	 */
	public IntakeActive(Intake intake, Indexer indexer) {
		this.intake = intake;
		this.indexer = indexer;
		
		addRequirements(intake, indexer);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		indexer.enable();
		indexer.setSpeed(0.25);

        intake.solenoidsDown();
		intake.intakeOn(0.75);
		intake.intakeIn(0.75);

	}


	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		indexer.disable();

		intake.intakeOn(0);
		intake.intakeIn(0);
        intake.solenoidsUp();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}


}

