package frc.robot.commands.shooter.state;

import org.jetbrains.annotations.NotNull;

import frc.robot.Constants;
import frc.robot.base.Property;
import frc.robot.base.command.InterruptType;
import frc.robot.base.command.TimedStateCommand;
import frc.robot.base.shooter.ShooterState;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.LedMode;
import frc.robot.subsystems.Limelight.TargetType;

// TODO update doc

/**
 * <h3>Operates the turret in the "Searching" state.</h3>
 * 
 * <p> In this state, the turret waits for the limelight led's to
 * turn on and checks to see if there are any targets. There's always
 * a chance that the limelight may detect a target the first time it's
 * led's turn since the robot should be generally facing it's target
 * upon shooting, and this state checks for that possibility. If there
 * is no target by the time the aqusition delay time runs, it switches
 * over to the {@code kSweeping} state.</p>
 */
public class SearchShooterState extends TimedStateCommand {
    private final Property<ShooterState> shooterState;

    private final Limelight limelight;
    
    public SearchShooterState(
        Limelight limelight,
        Property<ShooterState> shooterState
    ) {
        this.shooterState = shooterState;
        this.limelight = limelight;

        addRequirements(limelight);
    }

    @Override
    public void initialize() {
        super.initialize();

        limelight.enable();
        limelight.setLedMode(LedMode.kModeOn);

        shooterState.set(ShooterState.kSearch);
    }

    @Override
    public void execute() {
        if (limelight.getTargetType() == TargetType.kHub) {
            next("frc.robot.shooter.operate:dormant");
        } else if (getElapsedTime() > Constants.Vision.ACQUISITION_DELAY) {
            next("frc.robot.shooter.sweep");
        }
    }
    
    @Override
    public void end(InterruptType interrupt) {
        if (interrupt == InterruptType.kCancel || getNextState() == null)
            limelight.disable();
    }

    @Override
    public @NotNull String getName() {
        return "frc.robot.shooter.search";
    }
}
