package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeIndexer;

public class TestIndexProto extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private IntakeIndexer sys_intakeIndexer;

    public TestIndexProto(IntakeIndexer subsystem){
        sys_intakeIndexer = subsystem; 

        addRequirements(subsystem);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        sys_intakeIndexer.indexShootOn(1);
        sys_intakeIndexer.indexBeltOn(1);
    }

    @Override
    public void end(boolean interuppted){}

    @Override
    public boolean isFinished(){return false;}
    
    
}
