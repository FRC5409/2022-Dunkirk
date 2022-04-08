package frc.robot.commands.training.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.base.shooter.ShooterTrainingModel4;
import frc.robot.base.shooter.TrainingModelProvider;
import frc.robot.base.training.Setpoint;
import frc.robot.base.training.SetpointType;
import frc.robot.base.training.TrainerDashboard;

public class FlipTargetSetpoint extends CommandBase {
    private final TrainingModelProvider provider;
    private final TrainerDashboard dashboard;

    public FlipTargetSetpoint(TrainingModelProvider provider, TrainerDashboard dashboard) {
        this.dashboard = dashboard;
        this.provider = provider;
    }

    @Override
    public void initialize() { 
        ShooterTrainingModel4 executionModel = provider.getExecutionModel();
        
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
