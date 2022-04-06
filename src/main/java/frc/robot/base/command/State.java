package frc.robot.base.command;

import java.util.Map;
import java.util.Set;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * <h2> StateCommand </h2>
 * A state machine representing a complete action to be performed by the robot. Commands are run by
 * the {@link CommandScheduler}, and can be composed into CommandGroups to allow users to build
 * complicated multi-step actions without the need to roll the state machine logic themselves.
 *
 * <p>Commands are run synchronously from the main robot loop; no multithreading is used, unless
 * specified explicitly from the command implementation.
 *
 * <p>This class is provided by the NewCommands VendorDep
 *
 * 
 * @author Keith Davies
 */
public interface State extends Sendable {
    default void initialize() {}
    default void execute() {}
    default void end(InterruptType interrupt) {}

    default boolean isFinished() {
        return false;
    }
    
    public Set<Subsystem> getRequirements();
    
    void next(String stateName);

    void reset();

    default State addStates(State... states) {
        StateCommandManager.getInstance().addStateChildren(this, states);
        return this;
    }

    void setExecutionIndex(int index);

    @Nullable
    default State getParent() {
        return StateCommandManager.getInstance().getStateParent(this);
    }

    default Map<String, State> getChildren() {
        return StateCommandManager.getInstance().getStateChildren(this);
    }
    
    @Nullable
    String getNextState();
    
    @NotNull
    String getName();

    default String getPath() {
        return StateCommandManager.getInstance().getStatePath(this);
    }

    int getExecutionIndex();

    @Override
    default void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Path", this::getPath, null);
        builder.addStringProperty("Name", this::getName, null);
        builder.addStringProperty("Next State", () -> String.valueOf(this.getNextState()), null);
        builder.addDoubleProperty("Execution Index", this::getExecutionIndex, null);

    }
}
