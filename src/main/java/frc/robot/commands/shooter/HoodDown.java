package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.ShooterTurret;

/**
 * Command for putting the down. 
 * 
 * @author Akil Pathiranage.
 */
public class HoodDown extends CommandBase{

    ShooterTurret sys_turret;

    /**
     * Constructor
     * @param sys_turret ShooterTurret subsystem. 
     */
    public HoodDown(ShooterTurret sys_turret){
        addRequirements(sys_turret);
        this.sys_turret =  sys_turret;
    }

    @Override
    public void execute() {
        sys_turret.hoodDownPosition();
    }

    
    @Override
    public boolean isFinished() {
        return true;
    }
    
}
