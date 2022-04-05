package frc.robot.commands.training.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.base.training.Setpoint;
import frc.robot.base.training.SetpointType;
import frc.robot.base.training.TrainerContext;
import frc.robot.base.training.TrainerDashboard;
import frc.robot.base.training.TrainingModel4;

public class FlipTargetSetpoint extends CommandBase {
    private final TrainerContext context;
    private final TrainerDashboard dashboard;

    public FlipTargetSetpoint(TrainerDashboard dashboard, TrainerContext context) {
        this.context = context;
        this.dashboard = dashboard;
    }

    @Override
    public void execute() {    
        TrainingModel4 executionModel = context.getConfiguration().getExecutionModel();
        
        Setpoint target = executionModel.getSetpoint();
        if (target.getType() == SetpointType.kLeft)
            executionModel.setSetpoint(target.getParent().branch(SetpointType.kRight));
        else if (target.getType() == SetpointType.kRight)
            executionModel.setSetpoint(target.getParent().branch(SetpointType.kLeft));
        
        dashboard.update();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
