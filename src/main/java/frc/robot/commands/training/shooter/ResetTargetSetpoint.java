package frc.robot.commands.training.shooter;

import frc.robot.Constants;
import frc.robot.base.training.Setpoint;
import frc.robot.base.training.TrainerContext;
import frc.robot.base.training.TrainerDashboard;
import frc.robot.base.training.TrainingConfiguration;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ResetTargetSetpoint extends CommandBase {
    private final TrainerContext context;
    private final TrainerDashboard dashboard;

    public ResetTargetSetpoint(TrainerDashboard dashboard, TrainerContext context) {
        this.context = context;
        this.dashboard = dashboard;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        TrainingConfiguration configuration = context.getConfiguration();

        configuration.getExecutionModel()
            .setSetpoint(new Setpoint(Constants.Shooter.SPEED_RANGE.mid(), Constants.Shooter.SPEED_RANGE));
        
        dashboard.update();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
