package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.base.Property;
import frc.robot.subsystems.Indexer;

public class PrimeShooter extends CommandBase {
    private final Property<Boolean> armed;
    private final Indexer indexer;

    public PrimeShooter(Indexer indexer, Property<Boolean> armed) {
        this.indexer = indexer;
        this.armed = armed;
    }

    @Override
    public void initialize() {
        indexer.enable();
        indexer.setSpeed(-0.5);
        armed.set(false);
    }


    @Override
    public void end(boolean interrupted) {
        indexer.disable();
        armed.set(true);
    }
}
