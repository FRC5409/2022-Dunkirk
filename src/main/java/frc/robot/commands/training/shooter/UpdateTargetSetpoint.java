package frc.robot.commands.training.shooter;

import frc.robot.Constants;
import frc.robot.base.training.Setpoint;
import frc.robot.base.training.TrainerContext;
import frc.robot.base.training.TrainerDashboard;
import frc.robot.base.training.TrainingConfiguration;
import frc.robot.base.training.TrainingModel4;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class UpdateTargetSetpoint extends CommandBase {
    private final TrainerDashboard dashboard;
    private final TrainerContext context;

    public UpdateTargetSetpoint(TrainerDashboard dashboard, TrainerContext context) {
        this.dashboard = dashboard;
        this.context = context;
    }

    @Override
    public void initialize() {
        TrainingConfiguration configuration = context.getConfiguration();
        TrainingModel4 executionModel = configuration.getExecutionModel();

        executionModel.setSetpoint(
            new Setpoint(
                configuration.getExecutionModel().calculateReal(configuration.getDistance()),
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
