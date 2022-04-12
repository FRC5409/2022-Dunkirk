package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.Constants;
import frc.robot.base.Joystick;
import frc.robot.base.ValueProperty;
import frc.robot.base.Joystick.ButtonType;
import frc.robot.base.shooter.ShooterConfiguration;
import frc.robot.base.shooter.ShooterMode;
import frc.robot.base.shooter.SweepDirection;
import frc.robot.commands.shooter.ConfigureShooter;
import frc.robot.commands.shooter.OperateShooter;
import frc.robot.commands.shooter.RotateTurret;
import frc.robot.commands.shooter.RunShooter;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.shooter.ShooterFlywheel;
import frc.robot.subsystems.shooter.ShooterTurret;


public class ToggleShooterElevator extends CommandBase {    
    private final ValueProperty<Boolean> climberActive;
    private final Subsystem[] subsystems;
    private ShooterTurret turret;


    public ToggleShooterElevator(ValueProperty<Boolean> climberActive, Subsystem... _subsystems) {
        this.climberActive = climberActive;
        subsystems = _subsystems;
        addRequirements(_subsystems);
    }

    @Override
    public void initialize() {
        for (Subsystem sys : subsystems) {
            if (sys instanceof ShooterTurret) {
                turret = (ShooterTurret)sys;

                turret.enable();

                turret.setRotationTarget(climberActive.get() ? 0.0 : 90.0);                
                break;
            }
        }

        climberActive.set(!climberActive.get());
        // SmartDashboard.putBoolean("Climber active", climberActive.get());
    }

    @Override
    public boolean isFinished() {
        return turret.isTargetReached();
    }
}
