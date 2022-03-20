package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;


public class IntakeActive extends CommandBase{
    private final Intake sys_Intake;
	private final Indexer sys_indexer;

	/**
	 * Creates a new IntakeIndexActive
	 * 
	 * Command to run the indexer
	 */
	public IntakeActive(Intake subsystem, Indexer indexer) {
		sys_Intake = subsystem;
		sys_indexer = indexer;
		
		addRequirements(sys_Intake, sys_indexer);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {

        sys_Intake.solenoidsDown();

		sys_Intake.intakeOn(0.75);
		sys_indexer.enable();

	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		sys_indexer.spinIndexer(0.25);

        sys_Intake.intakeOn(0.75);
		sys_Intake.intakeIn(0.75);

	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		sys_Intake.intakeOn(0);
		sys_Intake.intakeIn(0);

        sys_Intake.solenoidsUp();
		sys_indexer.disable();

	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}


}

