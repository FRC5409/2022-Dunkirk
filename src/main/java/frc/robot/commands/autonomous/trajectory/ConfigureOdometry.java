package frc.robot.commands.autonomous.trajectory;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class ConfigureOdometry extends CommandBase {
    private final DriveTrain drivetrain;
    private final boolean enableOdometry;

    public ConfigureOdometry(DriveTrain drivetrain, boolean enableOdometry) {
        this.enableOdometry = enableOdometry;
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        drivetrain.setOdometryEnabled(enableOdometry);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
