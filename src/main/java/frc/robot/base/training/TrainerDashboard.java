package frc.robot.base.training;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TrainerDashboard {
    private final TrainerContext context;

    public TrainerDashboard(TrainerContext context) {
        this.context = context;
    }

    public void update() {
        TrainingConfiguration configuration = context.getConfiguration();

        SmartDashboard.putString("Trainer - Mode", context.getMode().name());
        SmartDashboard.putData("Trainer - Execution Model", configuration.getExecutionModel());
        SmartDashboard.putData("Trainer - Odometry Model", configuration.getOdometryModel());

        TrainingModel4 model = configuration.getExecutionModel();

        Setpoint setpoint = model.getSetpoint();

        SmartDashboard.putNumber("Trainer - Setpoint Target", setpoint.getTarget());
        SmartDashboard.putString("Trainer - Setpoint Type", setpoint.getType().name());
        SmartDashboard.putNumber("Trainer - Setpoint Range Max", setpoint.getRange().max());
        SmartDashboard.putNumber("Trainer - Setpoint Range Min", setpoint.getRange().min());
        SmartDashboard.putNumber("Trainer - Estimated Distance", configuration.getDistance());
        SmartDashboard.putNumber("Trainer - Estimated Target", model.calculateReal(configuration.getDistance()));
    }
}
