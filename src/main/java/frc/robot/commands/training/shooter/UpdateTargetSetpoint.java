package frc.robot.commands.training.shooter;

import frc.robot.Constants;
import frc.robot.base.shooter.TrainingModelProvider;
import frc.robot.base.training.Setpoint;
import frc.robot.base.training.TrainerDashboard;
import frc.robot.base.training.TrainingModel4;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class UpdateTargetSetpoint extends CommandBase {
    private final TrainingModelProvider provider;
    private final TrainerDashboard dashboard;

    public UpdateTargetSetpoint(TrainingModelProvider provider, TrainerDashboard dashboard) {
        this.dashboard = dashboard;
        this.provider = provider;
    }

    @Override
    public void initialize() {
        TrainingModel4 executionModel = provider.getExecutionModel();

        executionModel.setSetpoint(
            new Setpoint(
                executionModel.calculateReal(executionModel.getDistance()),
                Constants.Shooter.SPEED_RANGE
            )
        );
        
        dashboard.update();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
