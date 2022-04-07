package frc.robot.commands.training.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.base.shooter.TrainingModelProvider;
import frc.robot.base.training.Setpoint;
import frc.robot.base.training.TrainerDashboard;
import frc.robot.base.training.TrainingModel4;

public class UndoTargetSetpoint extends CommandBase {
    private final TrainingModelProvider provider;
    private final TrainerDashboard dashboard;

    public UndoTargetSetpoint(TrainingModelProvider provider, TrainerDashboard dashboard) {
        this.dashboard = dashboard;
        this.provider = provider;
    }

    @Override
    public void initialize() {
        TrainingModel4 executionModel = provider.getExecutionModel();
        
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
