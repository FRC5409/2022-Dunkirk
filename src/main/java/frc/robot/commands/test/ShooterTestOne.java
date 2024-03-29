package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.base.shooter.HoodPosition;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.shooter.ShooterFlywheel;
import frc.robot.subsystems.shooter.ShooterTurret;

public class ShooterTestOne extends CommandBase{
    private final ShooterFlywheel flywheel;
    private final ShooterTurret turret;
    private final Indexer indexer;

    public ShooterTestOne(ShooterFlywheel flywheel, ShooterTurret turret, Indexer indexer) {
        this.flywheel = flywheel;
        this.turret = turret;
        this.indexer = indexer;

        addRequirements(flywheel, turret, indexer);
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
        flywheel.setFeederVelocity(0.8);
        
        turret.setHoodPosition(HoodPosition.kDown);

        indexer.setSpeed(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.disable();
        turret.disable();
        indexer.disable();
    }
}
