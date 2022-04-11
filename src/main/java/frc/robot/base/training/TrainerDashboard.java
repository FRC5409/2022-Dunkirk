package frc.robot.base.training;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.base.shooter.ShooterTrainingModel4;
import frc.robot.base.shooter.TrainingModelProvider;

public class TrainerDashboard {
    private final TrainingModelProvider provider;

    public TrainerDashboard(TrainingModelProvider provider) {
        this.provider = provider;

        SmartDashboard.putData("Trainer - Execution Model", provider.getExecutionModel());
        SmartDashboard.putData("Trainer - Odometry Model", provider.getOdometryModel());
    }

    public void update() {
        ShooterTrainingModel4 model = provider.getExecutionModel();

        Setpoint setpoint = model.getSetpoint();

        SmartDashboard.putNumber("Trainer - Setpoint Target", setpoint.getTarget());
        SmartDashboard.putString("Trainer - Setpoint Type", setpoint.getType().name());
        SmartDashboard.putNumber("Trainer - Setpoint Range Max", setpoint.getRange().max());
        SmartDashboard.putNumber("Trainer - Setpoint Range Min", setpoint.getRange().min());
        SmartDashboard.putNumber("Trainer - Estimated Distance", model.getDistance());
        SmartDashboard.putNumber("Trainer - Estimated Target", model.calculateReal(model.getDistance()));
    }
}
