package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;

public class AutoAlign extends SequentialCommandGroup {
    public AutoAlign(Climber climber, DriveTrain driveTrain, double toAngle) {
        super();

        // addRequirements(driveTrain, pigeon);

        addCommands(
                new TurnToAngleGyro(driveTrain, toAngle),
                new DriveToMidRung(climber, driveTrain),
                new MoveToPosition(climber, driveTrain),
                // TODO: Fix later
                // new ElevateTo(climber, Constants.kClimber.TO_MAX),
                new MoveToPosition(climber, driveTrain, 0.2));
                // new ElevateTo(climber, Constants.kClimber.TO_MIN_MID));
    }
}
