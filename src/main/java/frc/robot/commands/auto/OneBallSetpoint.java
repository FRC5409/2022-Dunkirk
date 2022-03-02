package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IndexerIntakeActive;
import frc.robot.commands.MoveToDistance;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class OneBallSetpoint extends SequentialCommandGroup {

    private final DriveTrain driveTrain;
    private final Intake intake;
    private final Indexer indexer;
    // private final Shooter?

    public OneBallSetpoint(DriveTrain driveTrain, Intake intake, Indexer indexer) {
        
        this.driveTrain = driveTrain;
        this.intake     = intake;
        this.indexer    = indexer;

        addCommands(
            //shoot
        );


        addRequirements(driveTrain, indexer, intake);
        
    }
}