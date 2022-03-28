package frc.robot.commands.training.shooter;

import frc.robot.training.SetpointType;
import frc.robot.training.TrainerDashboard;
import frc.robot.training.TrainerContext;

import edu.wpi.first.wpilibj2.command.CommandBase;

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
