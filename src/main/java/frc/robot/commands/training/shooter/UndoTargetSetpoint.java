package frc.robot.commands.training.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.base.training.Setpoint;
import frc.robot.base.training.TrainerContext;
import frc.robot.base.training.TrainerDashboard;

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
