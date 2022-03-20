package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.base.Property;
import frc.robot.subsystems.Indexer;

public class PrimeShooter extends CommandBase {
    private final Property<Boolean> armed;
    private final Indexer indexer;
    private final Timer timer;

    public PrimeShooter(Indexer indexer, Property<Boolean> armed) {
        this.indexer = indexer;
        this.armed = armed;
        this.timer = new Timer();
    }

    @Override
    public void initialize() {
        indexer.enable();
        indexer.setSpeed(-1);
        armed.set(false);

        timer.reset();
    }


    @Override
    public void end(boolean interrupted) {
        indexer.disable();

        if (!interrupted)
            armed.set(true);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(Constants.Shooter.ARMING_TIME);
    }
    
}
