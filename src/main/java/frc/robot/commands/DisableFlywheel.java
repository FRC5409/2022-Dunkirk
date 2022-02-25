package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.ShooterFlywheel;

public class DisableFlywheel extends CommandBase{

    private ShooterFlywheel flywheel;
    public DisableFlywheel(ShooterFlywheel flywheel){
        this.flywheel = flywheel;
        addRequirements(this.flywheel);
    }


    @Override
    public void initialize() {
        System.out.println("Disabled");
        flywheel.disable();
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}
