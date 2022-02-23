package frc.robot.commands.training;

import frc.robot.Constants;
import frc.robot.training.Setpoint;
import frc.robot.training.TrainerDashboard;
import frc.robot.training.TrainerContext;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ResetTargetSetpoint extends CommandBase {
    private final TrainerContext _context;
    private final TrainerDashboard _dasboard;

    public ResetTargetSetpoint(TrainerDashboard dashboard, TrainerContext context) {
        _context = context;
        _dasboard = dashboard;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        _context.setSetpoint(new Setpoint(Constants.Shooter.SPEED_RANGE.mid(), Constants.Shooter.SPEED_RANGE));
        _dasboard.update();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
