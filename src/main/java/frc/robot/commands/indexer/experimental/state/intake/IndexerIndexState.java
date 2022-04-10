package frc.robot.commands.indexer.experimental.state.intake;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.base.Joystick;
import frc.robot.base.Property;
import frc.robot.base.command.InterruptType;
import frc.robot.base.command.StateBase;
import frc.robot.base.indexer.IndexerState;
import frc.robot.base.shooter.ShooterState;
import frc.robot.commands.joystick.JoystickRumble;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer.SensorType;

public class IndexerIndexState extends StateBase {
    private final Property<IndexerState> indexerState;

    private final Command rumbleCommand;

    private final Indexer indexer;
    private final Intake intake;

    private boolean tofEnter;

    public IndexerIndexState(
        Indexer indexer,
        Intake intake, 
        @Nullable Joystick joystick,
        Property<IndexerState> indexerState
    ) {
        this.indexerState = indexerState;
        this.indexer = indexer;
        this.intake = intake;

        if (joystick != null) {
            rumbleCommand = new JoystickRumble(0.5)
                .addJoysticks(joystick)
                .withDebounce(0.1)
                .withTimeout(0.5);
        } else {
            rumbleCommand = null;
        }

        addRequirements(indexer, intake);
    }

    @Override
    public void initialize() {
        indexer.enable();
        indexer.setSpeed(0.75);

        intake.intakeOn(0.4);
        
        tofEnter = indexer.getSensorState(SensorType.kEnter);

        indexerState.set(IndexerState.kActive);
    }
    
    @Override
    public void execute() {
        boolean tofEnter = indexer.getSensorState(SensorType.kEnter);
        boolean tofBall1 = indexer.getSensorState(SensorType.kBall1);
        boolean tofExit = indexer.getSensorState(SensorType.kExit);

        if (rumbleCommand != null) {
            if (this.tofEnter != tofEnter && tofEnter) {
                if (!rumbleCommand.isScheduled())
                    rumbleCommand.schedule();
            }
        }
        
        if (tofBall1 && tofExit) {
            indexer.setSpeed(0);
        } else {
            indexer.setSpeed(0.75);
        }

        this.tofEnter = tofEnter;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(InterruptType interrupt) {
        if (interrupt == InterruptType.kCancel) {
            intake.intakeOn(0);
            intake.intakeIn(0);
        }
        
        indexer.disable();

        if (rumbleCommand != null)
            rumbleCommand.cancel();
    }

    @Override
    public @NotNull String getName() {
        return "index";
    }
}
