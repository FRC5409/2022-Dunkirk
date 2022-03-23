package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class IndexerIntakeTest extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final Indexer indexer;
    private final Intake intake;

    public IndexerIntakeTest(Indexer indexer, Intake intake) {
        this.indexer = indexer;
        this.intake = intake;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(indexer, intake);
    }

    public void initialize(){
        indexer.enable();
        indexer.setSpeed(1);

        intake.intakeOn(0.5);
        intake.intakeIn(1);
        intake.solenoidsDown();
    }

    @Override
    public void end(boolean interuppted) {
        indexer.disable();

        intake.intakeOn(0);
        intake.intakeIn(0);
        intake.solenoidsUp();
    }
}
