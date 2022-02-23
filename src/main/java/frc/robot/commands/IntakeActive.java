package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;


import frc.robot.subsystems.Intake;


public class IntakeActive extends CommandBase{
    private final Intake sys_Intake;

	/**
	 * Creates a new IntakeIndexActive
	 * 
	 * Command to run the indexer
	 */
	public IntakeActive(Intake subsystem) {
		sys_Intake = subsystem;
		
		addRequirements(sys_Intake);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {

        sys_Intake.solenoidsDown();

		sys_Intake.intakeOn(0.75);

	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {

        sys_Intake.intakeOn(0.75);
		sys_Intake.intakeIn(0.75);

	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		sys_Intake.intakeOn(0);
		sys_Intake.intakeIn(0);

        sys_Intake.solenoidsUp();

	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}


}

