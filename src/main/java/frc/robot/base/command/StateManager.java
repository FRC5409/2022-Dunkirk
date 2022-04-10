package frc.robot.base.command;

import java.util.WeakHashMap;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.jetbrains.annotations.Nullable;

public class StateManager {
    private static final StateManager m_instance = new StateManager();
    
    public static final StateManager getInstance() {
        return m_instance;
    }

    private final Map<State, Map<String, State>> m_children;
    private final Map<State, State> m_ancestor;
    private final Map<State, State> m_parent;
    private final Map<State, String> m_path;

    public StateManager() {
        m_ancestor = new WeakHashMap<>();
        m_children = new WeakHashMap<>();
        m_parent = new WeakHashMap<>();
        m_path = new WeakHashMap<>();
    }

    public void addState(State state) {
        m_parent.put(state, null);
        m_ancestor.put(state, state);
        m_children.put(state, new HashMap<>());
    }

    public void addStateChildren(State parent, State[] states) {
        addStateChildren(parent, List.of(states));
    }

    public void addStateChildren(State parent, Iterable<State> states) {
        if (!exists(parent))
            throw new IllegalArgumentException("State does not exist on manager.");

        State ancestor = m_ancestor.get(parent);

        Map<String, State> children = m_children.get(parent);
        for (State child : states) {
            if (!exists(child))
                throw new IllegalArgumentException("Child state '" + child.getName() + "' does not exist on manager.");
            
            if (children.containsKey(child.getName())) {
                throw new IllegalArgumentException("Child state '" + child.getName()
                    + "' already exists on parent state '" + parent.getName());
            } else {
                State childParent = getStateParent(child);
                if (childParent != null) {
                    if (childParent.equals(parent))
                        continue;
                        
                    throw new IllegalArgumentException("Child state '" + child.getName()
                        + "' already has parent state '" + childParent.getName());
                }
            } 

            children.put(child.getName(), child);
            m_ancestor.put(child, ancestor);
        }
    }
    
    @Nullable
    public List<State> getStatesOnPath(State root, String[] path) {
        return getStatesOnPath(root, path, 0);   
    }

    @Nullable
    public List<State> getStatesOnPath(State root, String[] path, int startIndex) {
        if (!exists(root))
            throw new IllegalArgumentException("State does not exist on manager.");
        else if (path.length == 0)
            throw new IllegalArgumentException("Cannot search for states on empty path.");
        else if (startIndex >= path.length)
            throw new IllegalArgumentException("Start index is greater than path length");

        State base = root;
        Map<String, State> children = m_children.get(base);

        // State path does not exist within ancestry
        if (!children.containsKey(path[startIndex]))
            return null;

        List<State> states = new ArrayList<>();

        base = children.get(path[startIndex]);
        states.add(base);

        // Walk up state structure with path components
        for (int i = startIndex+1; i < path.length; i++) {
            children = m_children.get(base);

            if (!children.containsKey(path[i]))
                throw new RuntimeException("State with name '" + 
                    StateNameFormat.formatPath(path) + "' does not exist on state '" + getStatePath(base) + "'");

            base = children.get(path[i]);
            states.add(base);
        }

        return states;
    }

    @Nullable
    public State getStateParent(State state) {
        if (!exists(state))
            throw new IllegalArgumentException("State does not exist on manager.");
        
        return m_parent.get(state);
    }

    public Map<String, State> getStateChildren(State state) {
        if (!exists(state))
            throw new IllegalArgumentException("State does not exist on manager.");
        return Collections.unmodifiableMap(m_children.get(state));
    }

    public String getStatePath(State state) {
        if (!exists(state))
            throw new IllegalArgumentException("State does not exist on manager.");
        
        // Lazily calculate state path
        if (!m_path.containsKey(state)) {
            List<State> ancestry = new ArrayList<>();
            ancestry.add(state);

            State parent = getStateParent(state);
            while (parent != null) {
                ancestry.add(parent);
                parent = getStateParent(parent);
            }

            StringBuilder builder = new StringBuilder();
            for (int i = ancestry.size()-1; i >= 0; i--) {
                builder.append(ancestry.get(i).getName());
                builder.append(StateNameFormat.SEPERATOR);
            }

            builder.deleteCharAt(builder.length()-1);

            String path = builder.toString();

            m_path.put(state, path);

            return path;
        } else {
            return m_path.get(state);
        }
    }

    public boolean exists(State state) {
        return m_parent.containsKey(state);
    }
}
