package frc.robot.commands.training.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.base.training.Setpoint;
import frc.robot.base.training.TrainerContext;
import frc.robot.base.training.TrainerDashboard;
import frc.robot.base.training.TrainingConfiguration;
import frc.robot.base.training.TrainingModel4;

public class UndoTargetSetpoint extends CommandBase {
    private final TrainerContext context;
    private final TrainerDashboard dashboard;

    public UndoTargetSetpoint(TrainerDashboard dashboard, TrainerContext context) {
        this.context = context;
        this.dashboard = dashboard;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        TrainingModel4 executionModel = context.getConfiguration()
            .getExecutionModel();
        
        Setpoint targetParent = executionModel.getSetpoint().getParent();
        if (targetParent != null)
            executionModel.setSetpoint(targetParent);
        
        dashboard.update();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
