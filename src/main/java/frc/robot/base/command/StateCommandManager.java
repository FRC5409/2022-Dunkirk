package frc.robot.base.command;

import java.util.WeakHashMap;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.jetbrains.annotations.Nullable;

public class StateCommandManager {
    private static final StateCommandManager m_instance = new StateCommandManager();
    
    public static final StateCommandManager getInstance() {
        return m_instance;
    }

    private final Map<StateCommand, StateCommand> m_parent;
    private final Map<StateCommand, StateCommand> m_ancestor;
    private final Map<StateCommand, Map<String, StateCommand>> m_children;

    public StateCommandManager() {
        m_parent = new WeakHashMap<>();
        m_ancestor = new WeakHashMap<>();
        m_children = new WeakHashMap<>();
    }

    public void addState(StateCommand state) {
        m_parent.put(state, null);
        m_ancestor.put(state, state);
        m_children.put(state, new HashMap<>());
    }

    public void addStateChildren(StateCommand parent, StateCommand[] states) {
        addStateChildren(parent, List.of(states));
    }

    public void addStateChildren(StateCommand parent, Iterable<StateCommand> states) {
        if (!exists(parent))
            throw new IllegalArgumentException("State does not exist on manager.");

        StateCommand ancestor = m_ancestor.get(parent);

        Map<String, StateCommand> children = m_children.get(parent);
        for (StateCommand child : states) {
            if (!exists(child))
                throw new IllegalArgumentException("Child state '" + child.getStateName() + "' does not exist on manager.");
            
            if (children.containsKey(child.getStateName())) {
                throw new IllegalArgumentException("Child state '" + child.getStateName()
                    + "' already exists on parent state '" + parent.getStateName());
            } else {
                StateCommand childParent = getStateParent(child);
                if (childParent != null) {
                    if (childParent.equals(parent))
                        continue;
                        
                    throw new IllegalArgumentException("Child state '" + child.getStateName()
                        + "' already has parent state '" + childParent.getStateName());
                }
            } 

            children.put(child.getStateName(), child);
            m_ancestor.put(child, ancestor);
        }
    }
    
    @Nullable
    public List<StateCommand> getStatesOnPath(StateCommand root, String[] path) {
        return getStatesOnPath(root, path, 0);   
    }

    @Nullable
    public List<StateCommand> getStatesOnPath(StateCommand root, String[] path, int startIndex) {
        if (!exists(root))
            throw new IllegalArgumentException("State does not exist on manager.");
        else if (path.length == 0)
            throw new IllegalArgumentException("Cannot search for states on empty path.");
        else if (startIndex >= path.length)
            throw new IllegalArgumentException("Start index is greater than path length");

        StateCommand base = root;
        Map<String, StateCommand> children = m_children.get(base);

        // State path does not exist within ancestry
        if (!children.containsKey(path[0]))
            return null;

        List<StateCommand> states = new ArrayList<>();

        base = children.get(path[0]);
        states.add(base);

        // Walk up state structure with path components
        for (int i = 1; i < path.length; i++) {
            children = m_children.get(base);

            if (!children.containsKey(path[i]))
                throw new RuntimeException("State with name '" + StateFormat.formatPath(path) + "' does not exist.");

            base = children.get(path[i]);
            states.add(base);
        }

        return states;
    }

    @Nullable
    public StateCommand getStateParent(StateCommand state) {
        if (!exists(state))
            throw new IllegalArgumentException("State does not exist on manager.");
        
        return m_parent.get(state);
    }

    public Map<String, StateCommand> getStateChildren(StateCommand state) {
        if (!exists(state))
            throw new IllegalArgumentException("State does not exist on manager.");
        return Collections.unmodifiableMap(m_children.get(state));
    }

    public boolean exists(StateCommand state) {
        return m_parent.containsKey(state);
    }
}
