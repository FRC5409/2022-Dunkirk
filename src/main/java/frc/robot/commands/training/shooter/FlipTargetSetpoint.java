package frc.robot.commands.training.shooter;

import frc.robot.training.Setpoint;
import frc.robot.training.SetpointType;
import frc.robot.training.TrainerDashboard;
import frc.robot.training.TrainerContext;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class FlipTargetSetpoint extends CommandBase {
    private final TrainerContext _context;
    private final TrainerDashboard _dasboard;

    public FlipTargetSetpoint(TrainerDashboard dashboard, TrainerContext context) {
        _context = context;
        _dasboard = dashboard;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {    
        Setpoint target = _context.getSetpoint();
        
        if (target.getType() == SetpointType.kLeft)
            _context.setSetpoint(target.getParent().branch(SetpointType.kRight));
        else if (target.getType() == SetpointType.kRight)
            _context.setSetpoint(target.getParent().branch(SetpointType.kLeft));
        
        _dasboard.update();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
