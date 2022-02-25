package frc.robot.commands.training;

import frc.robot.training.BranchType;
import frc.robot.training.Setpoint;
import frc.robot.training.TrainerDashboard;
import frc.robot.training.TrainerContext;
import frc.robot.utils.Range;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class BranchTargetSetpoint extends CommandBase {
    private final TrainerContext _context;
    private final BranchType _type;
    private final TrainerDashboard _dasboard;

    public BranchTargetSetpoint(TrainerDashboard dashboard, TrainerContext context, BranchType type) {
        _context = context;
        _type = type;
        _dasboard = dashboard;
    }

    @Override
    public void execute() {
        Setpoint active = _context.getSetpoint();
        switch(_type) {
            case BRANCH_LEFT:
                _context.setSetpoint(active.branch(true));
                break;
            case BRANCH_RIGHT:
                _context.setSetpoint(active.branch(false));
                break;
            case BRANCH_CENTER:
                Range range = active.getRange();
                double target = active.getTarget();

                _context.setSetpoint(
                    new Setpoint(
                        target, new Range((target + range.min()) / 2, (target + range.max()) / 2)
                    )
                );
                
                break;
        }

        _dasboard.update();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
