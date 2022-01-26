package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeIndexer;

public class IntakeSimulationTesting extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private IntakeIndexer sys_intakeIndexer;

    public IntakeSimulationTesting(IntakeIndexer subsystem){
        sys_intakeIndexer = subsystem; 
        addRequirements(sys_intakeIndexer);
    }

    @Override
    public void initialize(){
        sys_intakeIndexer.solenoidsDown();
    }

    @Override
    public void execute(){
        sys_intakeIndexer.intakeOn(1);
        sys_intakeIndexer.solenoidsUp();
    }

    @Override
    public void end(boolean interrupted){
        sys_intakeIndexer.intakeOn(0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}
