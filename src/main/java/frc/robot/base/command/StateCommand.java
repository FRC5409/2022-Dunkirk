package frc.robot.base.command;

import java.util.Collection;
import java.util.Map;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import edu.wpi.first.wpilibj2.command.Command;

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
public interface StateCommand extends Command {
    void next(String stateName);

    void reset();

    default void addStates(StateCommand... states) {
        StateCommandManager.getInstance().addStateChildren(this, states);
    }
    
    default void addStates(Collection<StateCommand> states) {
        StateCommandManager.getInstance().addStateChildren(this, states);
    }

    void setExecutionIndex(int index);

    @Nullable
    String getNextState();
    
    @NotNull
    String getStateName();

    int getExecutionIndex();
    
    @Nullable
    default StateCommand getParent() {
        return StateCommandManager.getInstance().getStateParent(this);
    }

    default Map<String, StateCommand> getChildren() {
        return StateCommandManager.getInstance().getStateChildren(this);
    }
}
