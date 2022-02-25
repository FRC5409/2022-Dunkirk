package frc.robot.commands;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.kDriveTrain;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pigeon;

public class AutoRoutine extends SequentialCommandGroup{

    DriveTrain drive;
    Pigeon pigeon;
    Intake intake;
    Indexer indexer;

    public AutoRoutine(DriveTrain drive, Pigeon pigeon, Intake intake, Indexer indexer){

        this.drive   = drive;
        this.pigeon  = pigeon;
        this.intake  = intake;
        this.indexer = indexer;

        addCommands(
            new MoveToAngle(-30, drive, pigeon),
            new MoveToDistance(drive, -10),
            new MoveToAngle(-90, drive, pigeon),

            // added parallel commands to a new ParallelCommandGroup
            new ParallelCommandGroup().alongWith(
                    new IntakeActive(intake).withTimeout(1),
                    new MoveToDistance(drive, 10)
                ),

            new ReverseIntakeIndexer(intake, indexer).withTimeout(1)
        ); 
    }

    @Override
    public void initialize(){
        drive.setSpeedLimit(kDriveTrain.autoSpeed);
    }

    @Override
    public void end(boolean interrupt){
        drive.setSpeedLimit(kDriveTrain.teleopSpeed);
    }
}
