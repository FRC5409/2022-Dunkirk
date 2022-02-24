package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class IndexerIntakeTest extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private Indexer sys_indexer;
    private Intake sys_intake;

    public IndexerIntakeTest(Indexer indexer, Intake intake) {
        sys_indexer = indexer;
        sys_intake = intake;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(indexer, intake);
    }

    @Override
    public void execute() {
        sys_intake.intakeOn(0.5);
        sys_intake.intakeIn(1);
        sys_indexer.indexerOn(1);
        sys_intake.solenoidsDown();
    }

    @Override
    public void end(boolean interuppted) {
        sys_indexer.indexerOn(0);
        sys_intake.intakeOn(0);
        sys_intake.intakeIn(0);
        sys_intake.solenoidsUp();
    }
}
