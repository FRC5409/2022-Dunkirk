package frc.robot.commands.training;

import frc.robot.training.Setpoint;
import frc.robot.training.TrainerDashboard;
import frc.robot.training.TrainerContext;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class UndoTargetSetpoint extends CommandBase {
    private final TrainerContext _context;
    private final TrainerDashboard _dasboard;

    public UndoTargetSetpoint(TrainerDashboard dashboard, TrainerContext context) {
        _context = context;
        _dasboard = dashboard;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        Setpoint targetParent = _context.getSetpoint().getParent();
        if (targetParent != null)
            _context.setSetpoint(targetParent);
        
        _dasboard.update();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
