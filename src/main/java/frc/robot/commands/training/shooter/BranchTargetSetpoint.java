package frc.robot.commands.training.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.base.training.SetpointType;
import frc.robot.base.training.TrainerContext;
import frc.robot.base.training.TrainerDashboard;

public class BranchTargetSetpoint extends CommandBase {
    private final TrainerDashboard _dasboard;
    private final TrainerContext _context;
    private final SetpointType _type;

    public BranchTargetSetpoint(TrainerDashboard dashboard, TrainerContext context, SetpointType type) {
        _dasboard = dashboard;
        _context = context;
        _type = type;
    }

    @Override
    public void execute() {
        _context.setSetpoint(_context.getSetpoint().branch(_type));
        _dasboard.update();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
