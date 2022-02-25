package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.ShooterTurret;

/**
 * Comamnd for putting the hood up.
 * @author Akil Pathiranage. 
 */
public class HoodUp extends CommandBase{

    ShooterTurret sys_turret;

    /**
     * Constructor
     * @param sys_turret Turret subsystem. 
     */
    public HoodUp(ShooterTurret sys_turret){
        addRequirements(sys_turret);
        this.sys_turret =  sys_turret;
    }

    @Override
    public void execute() {
        sys_turret.hoodUpPosition();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
