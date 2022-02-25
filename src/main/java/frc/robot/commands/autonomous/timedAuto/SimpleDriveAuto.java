package frc.robot.commands.autonomous.timedAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveStraight;
import frc.robot.subsystems.DriveTrain;

public class SimpleDriveAuto extends SequentialCommandGroup {

    public SimpleDriveAuto(DriveTrain sys_drive) {
        addCommands(
            new DriveStraight(sys_drive, -0.2f, -0.2f).withTimeout(1)
        );
    }
}