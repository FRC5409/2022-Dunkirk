package frc.robot.commands.training.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.base.training.SetpointType;
import frc.robot.base.training.TrainerContext;
import frc.robot.base.training.TrainerDashboard;
import frc.robot.base.training.TrainingModel4;

public class BranchTargetSetpoint extends CommandBase {
    private final TrainerDashboard dashboard;
    private final TrainerContext context;
    private final SetpointType type;

    public BranchTargetSetpoint(TrainerDashboard dashboard, TrainerContext context, SetpointType type) {
        this.dashboard = dashboard;
        this.context = context;
        this.type = type;
    }

    @Override
    public void execute() {
        TrainingModel4 executionModel = context.getConfiguration().getExecutionModel();
        executionModel.setSetpoint(executionModel.getSetpoint().branch(type));
        dashboard.update();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
