package frc.robot.training;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.base.shooter.odometry.ShooterExecutionModel;

public class TrainerDashboard {
    private final TrainerContext context;

    public TrainerDashboard(TrainerContext context) {
        this.context = context;
    }

    public void update() {
        ShooterExecutionModel executionModel = context.getExecutionModel();
        Setpoint target = context.getSetpoint();

        SmartDashboard.putString("Trainer - Mode", context.getMode().name());
        SmartDashboard.putData("Trainer - Execution Model", executionModel);
        SmartDashboard.putData("Trainer - Odometry Model", context.getOdometryModel());

        SmartDashboard.putNumber("Trainer - Setpoint Target", target.getTarget());
        SmartDashboard.putString("Trainer - Setpoint Type", target.getType().name());
        SmartDashboard.putNumber("Trainer - Setpoint Range Max", target.getRange().max());
        SmartDashboard.putNumber("Trainer - Setpoint Range Min", target.getRange().min());
        SmartDashboard.putNumber("Trainer - Estimated Distance", context.getDistance());
        SmartDashboard.putNumber("Trainer - Estimated Target", executionModel.calculate(context.getDistance()));
    }
}
