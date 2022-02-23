package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeSimulationTesting extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private Intake sys_intake;

    public IntakeSimulationTesting(Intake subsystem){
        sys_intake = subsystem; 
        addRequirements(sys_intake);
    }

    @Override
    public void initialize(){
        sys_intake.solenoidsDown();
    }

    @Override
    public void execute(){
        sys_intake.intakeOn(1);
        sys_intake.solenoidsUp();
    }

    @Override
    public void end(boolean interrupted){
        sys_intake.intakeOn(0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}
