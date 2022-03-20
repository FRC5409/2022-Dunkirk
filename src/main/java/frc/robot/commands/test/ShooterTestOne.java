package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.shooter.ShooterFlywheel;
import frc.robot.subsystems.shooter.ShooterTurret;

public class ShooterTestOne extends CommandBase{
    ShooterFlywheel flywheel;
    ShooterTurret turret;
    Indexer indexer;

    public ShooterTestOne(ShooterFlywheel flywheel, ShooterTurret turret, Indexer indexer){
        this.flywheel = flywheel;
        this.turret = turret;
        this.indexer = indexer;
    }

    @Override
    public void initialize() {
        turret.enable();
        flywheel.enable();
        indexer.enable();
    }

    @Override
    public void execute() {
        flywheel.setVelocity(2500);
        flywheel.spinFeederSetpoint(0.8);
        turret.hoodDownPosition();
        indexer.spinIndexer(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        this.flywheel.disable();
        this.turret.disable();
        this.indexer.disable();
    }
}
