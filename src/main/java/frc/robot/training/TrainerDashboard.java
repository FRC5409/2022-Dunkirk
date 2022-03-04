package frc.robot.training;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.base.shooter.ShooterModel;

public class TrainerDashboard {
    private final TrainerContext _context;

    public TrainerDashboard(TrainerContext context) {
        _context = context;
    }

    public void update() {
        Setpoint target = _context.getSetpoint();
        SmartDashboard.putNumber("Setpoint Target", target.getTarget());
        SmartDashboard.putString("Setpoint Type", target.getType().name());
        SmartDashboard.putNumber("Setpoint Range Max", target.getRange().max());
        SmartDashboard.putNumber("Setpoint Range Min", target.getRange().min());

        ShooterModel model = _context.getModel();
        
        SmartDashboard.putString("Training Mode", _context.getMode().name());
        SmartDashboard.putNumber("Training Model kA", model.kA);
        SmartDashboard.putNumber("Training Model kB", model.kB);
        SmartDashboard.putNumber("Training Model kC", model.kC);
        SmartDashboard.putNumber("Training Model kD", model.kD);
        
        SmartDashboard.putNumber("Estimated Distance", _context.getDistance());
        SmartDashboard.putNumber("Estimated Target", model.calculate(_context.getDistance()));
    }
}
