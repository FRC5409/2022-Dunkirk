package frc.robot.commands;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pigeon;

public class AutoRoutine extends SequentialCommandGroup{
    public AutoRoutine(DriveTrain drive, Pigeon pigeon, Intake intake, Indexer indexer){
        addCommands(
            new MoveToDistance(drive, 10),
            // shoot
            new MoveToAngle(drive, 90),
            new MoveToDistance(drive, 10),
            // intake
            // index
            new MoveToAngle(drive, -90)
            // shoot
        );
    }
}
