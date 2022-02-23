package frc.robot.commands.training;

import frc.robot.Constants;
import frc.robot.training.Setpoint;
import frc.robot.training.SetpointType;
import frc.robot.training.TrainerDashboard;
import frc.robot.training.TrainerContext;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class UpdateTargetSetpoint extends CommandBase {
    private final TrainerContext _context;
    private final TrainerDashboard _dasboard;

    public UpdateTargetSetpoint(TrainerDashboard dashboard, TrainerContext context) {
        _context = context;
        _dasboard = dashboard;
    }

    @Override
    public void initialize() {
        _context.setSetpoint(
            new Setpoint(
                _context.getModel().calculate(_context.getDistance()),
                Constants.Shooter.SPEED_RANGE
            )
        );
        
        _dasboard.update();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
