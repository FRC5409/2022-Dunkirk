package frc.robot.commands;

import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ReverseIntake extends CommandBase {
  private final Intake sys_Intake;

    /**
     * Creates a new ReverseIntake.
     */
    public ReverseIntake(Intake subsystem) {
        sys_Intake = subsystem;
        addRequirements(sys_Intake);
    }

    // Called when the command is initially scheduled.
    @Override
        public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        sys_Intake.reverseIntake(0.5);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        sys_Intake.intakeOn(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}