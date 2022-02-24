package frc.robot.commands.training;

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
        
        if (target.getType() == SetpointType.LEFT)
            _context.setSetpoint(target.getParent().branch(false));
        else if (target.getType() == SetpointType.RIGHT)
            _context.setSetpoint(target.getParent().branch(true));
        
        _dasboard.update();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
