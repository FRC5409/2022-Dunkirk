package frc.robot.commands.shooter;

import org.jetbrains.annotations.Nullable;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.base.Property;
import frc.robot.base.shooter.ShooterConfiguration;
import frc.robot.base.shooter.ShooterModel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.TargetType;
import frc.robot.subsystems.shooter.ShooterFlywheel;
import frc.robot.subsystems.shooter.ShooterTurret;
import frc.robot.utils.Vector2;

public class Shoot extends CommandBase{

    private ShooterFlywheel flywheel;
    private Indexer indexer;
    private Limelight limelight;
    private Property<ShooterConfiguration> config;
    private ShooterModel model;
    private boolean active;


    public Shoot(ShooterFlywheel flywheel,Indexer indexer, Limelight limelight, Property<ShooterConfiguration> configuration){
        this.flywheel = flywheel;
        this.indexer = indexer;
        this.limelight = limelight;
        config = configuration;
        active = false;
        addRequirements(flywheel,indexer);
    }

    @Override
    public void initialize() {
        flywheel.enable();
        indexer.enable();
        limelight.enable();

        flywheel.spinFeeder(Constants.Shooter.FEEDER_VELOCITY);
        model = config.get().getModel();

        if(model != null || (limelight.hasTarget() && limelight.getTargetType() == TargetType.kHub)){
            Vector2 target = limelight.getTarget();
            double distance = model.distance(target.y);
            double velocity = model.calculate(distance);
            flywheel.setVelocity(velocity);
        }
    }

    @Override
    public void execute() {
        if(model == null || !(limelight.hasTarget() && limelight.getTargetType() == TargetType.kHub)) return;

        //TODO offset has been removed

        // System.out.println("Feeder reached target: " + flywheel.feederReachedTarget());
        // System.out.println("Feeder RPM: " + flywheel.getFeederRpm());
        // System.out.println("Target: " + flywheel.getFeederTarget());
        if (!active && flywheel.isTargetReached() && flywheel.feederReachedTarget()) {
            // System.out.println("Targets Reached");
            indexer.indexerOn(1);
            active = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.disable();
        indexer.disable();
        limelight.disable();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
