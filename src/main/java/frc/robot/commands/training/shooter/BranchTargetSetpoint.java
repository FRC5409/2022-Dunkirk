package frc.robot.commands.training.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.base.shooter.ShooterTrainingModel4;
import frc.robot.base.shooter.TrainingModelProvider;
import frc.robot.base.training.SetpointType;
import frc.robot.base.training.TrainerDashboard;

public class BranchTargetSetpoint extends CommandBase {
    private final TrainingModelProvider provider;
    private final TrainerDashboard dashboard;
    private final SetpointType type;

    public BranchTargetSetpoint(TrainingModelProvider provider, TrainerDashboard dashboard, SetpointType type) {
        this.dashboard = dashboard;
        this.provider = provider;
        this.type = type;
    }

    @Override
    public void execute() {
        ShooterTrainingModel4 executionModel = provider.getExecutionModel();
        executionModel.setSetpoint(executionModel.getSetpoint().branch(type));
        dashboard.update();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
