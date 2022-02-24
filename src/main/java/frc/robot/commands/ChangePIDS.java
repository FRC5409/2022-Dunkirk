package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.ShooterFlywheel;

public class ChangePIDS extends CommandBase{

    private ShooterFlywheel flywheel;

    public ChangePIDS(ShooterFlywheel flywheel){
        this.flywheel = flywheel;
        addRequirements(this.flywheel);
    }

    @Override
    public void initialize() {
        System.out.println("Pid configured");
        flywheel.configPIDS();
    }
    

    @Override
    public boolean isFinished() {
        return true;
    }
}
