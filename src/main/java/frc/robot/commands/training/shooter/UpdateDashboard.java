package frc.robot.commands.training.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.base.training.TrainerDashboard;

public class UpdateDashboard extends CommandBase {
    private final TrainerDashboard dashboard;

    public UpdateDashboard(TrainerDashboard dashboard) {
        this.dashboard = dashboard;
    }
    
    @Override
    public void execute() {
        dashboard.update();
    }
}
