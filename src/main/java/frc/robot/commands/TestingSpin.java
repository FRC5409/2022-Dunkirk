package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.ShooterFlywheel;

public class TestingSpin extends CommandBase{

    ShooterFlywheel flywheel;
    
    public TestingSpin(ShooterFlywheel flywheel){
        this.flywheel = flywheel;
        addRequirements(this.flywheel);
    }

    @Override
    public void initialize() {
        System.out.println("Speed updated");
        flywheel.testVelocity();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
