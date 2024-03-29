package frc.robot.base.command;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.jetbrains.annotations.Nullable;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

class StateExecutionStack implements Command {
    private final List<State> m_states;
    private final Set<Subsystem> m_requirements;
    private final int m_branch;

    private StateExecutionStack m_nextStack;
    private String[] m_next;
    private boolean m_dirty;
    private int m_exitor;

    private static Set<Subsystem> getStateRequirements(Iterable<State> states) {
        Set<Subsystem> requirements = new HashSet<>();
        for (State state : states)
            requirements.addAll(state.getRequirements());
        return requirements;
    }

    private static List<State> asGroup(State state) {
        List<State> states = new ArrayList<>();
        states.add(state);
        return states;
    }

    public StateExecutionStack(State state) {
        this(asGroup(state));
    }

    public StateExecutionStack(List<State> states) {
        this(states, getStateRequirements(states), 0);
    }

    private StateExecutionStack(List<State> states, Set<Subsystem> requirements, int branchIndex) {
        if (states.size() == 0)
            throw new IllegalArgumentException("Cannot create execution stack with no states");
        else if (branchIndex != -1 && branchIndex > states.size())
            throw new IllegalArgumentException("Cannot create execution stack with invalid branch index");
            

        m_requirements = requirements;
        m_nextStack = null;
        m_states = states;
        m_branch = branchIndex;
        m_dirty = false;
    }

    @Override
    public void initialize() {
        if (m_dirty) return;

        // Update indexes before state execution in case
        // they need correct values very early on
        updateIndexes();

        if (m_branch != -1) {
            // Initialize new states starting from branch index,
            // and preserve state execution of already running states
            for (int i = m_branch; i < m_states.size(); i++)
                m_states.get(i).initialize();
        } 

        
        m_exitor = -1;
        m_next = null;
    }

    @Override
    public void execute() {
        if (m_dirty) return;

        for (int i = 0; i < m_states.size(); i++) {
            State state = m_states.get(i);

            // Update state
            state.execute();

            // If state wishes to exit, mark it as the exitor
            // and prevent the update loop from continuing
            if ((state.isFinished() || state.getNextState() != null) && m_exitor == -1) {
                m_exitor = i;
                break;
            }
        }

        if (m_exitor != -1) {
            State exitorState = m_states.get(m_exitor);
            
            // Query next state
            String nextStatePath = exitorState.getNextState();
            if (nextStatePath != null && !nextStatePath.isEmpty()) {
                String[] path = StateNameFormat.parsePath(nextStatePath);

                // Get next states
                List<State> nextStates = StateCommandManager.getInstance()
                    .getStatesOnPath(exitorState, path);
                
                // Check if the state we are transitioning from contains
                // the next state within its local ancestry
                if (nextStates == null) {
                    // Walk backward up the stack to find the
                    // next state to transition from.
                    int idx = -1;
                    for (int i = m_exitor; i >= 0; i--) {
                        State state = m_states.get(i);

                        nextStates = StateCommandManager.getInstance()
                            .getStatesOnPath(state, path);

                        // Check if state path exists within its local ancestry
                        if (nextStates != null) {
                            idx = i;
                            break;
                        }
                    }
                   
                    // Because we had to walk back up the stack to find a pathway
                    // for the next transition, we effectively moved the exitor
                    // backwards, since we are now moving from this state to another now.
                    if (idx != -1) {
                        m_exitor = idx;
                        exitorState = m_states.get(idx);
                    }
                }

                // Interrupt any commands above the exitor on the stack
                // since they do not have authority over the exit
                for (int i = m_states.size()-1; i > m_exitor; i--) {               
                    State state = m_states.get(i);

                    state.end(InterruptType.kDetach);
                    state.reset();
    
                    m_states.remove(i);
                }
    
                // Check if the next states exist within the stack ancestry.
                // (If next state has the same ancestor as the exitor state of this stack)
                if (nextStates != null) {
                    m_states.addAll(nextStates);

                    exitorState.reset();
                    // Check if requirements change during state transition
                    Set<Subsystem> nextRequirements = getStateRequirements(m_states);
                    if (nextRequirements.equals(m_requirements)) {
                        updateIndexes();

                        // Continue execution on same stack and initialize new states
                        for (int i = m_exitor+1; i < m_states.size(); i++)
                            m_states.get(i).initialize();

                        m_exitor = -1;
                    } else {
                        // Branch onto new stack with new requirements, and discard
                        // current stack
                        m_nextStack = new StateExecutionStack(m_states, nextRequirements, m_exitor+1);
                        m_nextStack.schedule();

                        // Mark current stack as dirty
                        m_dirty = true;
                    }
                } else {
                    // The next states exist outside of the ancestry of this stack
                    for (int i = m_states.size()-1; i >= 0; i--) {
                        State state = m_states.get(i);

                        // End active states
                        if (i == m_exitor)
                            state.end(InterruptType.kTransition);
                        else
                            state.end(InterruptType.kDetach);

                        state.reset();
                    }

                    // Mark this stack as dirty
                    m_next = path;
                    m_dirty = true;
                }
            } else {
                for (int i = m_states.size()-1; i >= m_exitor; i--) {
                    State state = m_states.get(i);
    
                    // End active states
                    if (i == m_exitor)
                        state.end(InterruptType.kFinish);
                    else
                        state.end(InterruptType.kDetach);

                    state.reset();
                    
                    m_states.remove(i);
                }

                // If the exitor is the root of the stack,
                // mark this stack as dirty, ending its execution.
                if (m_exitor == 0) {
                    m_dirty = true;
                } else {
                    // Check if requirements changed
                    Set<Subsystem> nextRequirements = getStateRequirements(m_states);
                    if (nextRequirements.equals(m_requirements)) {
                        // Continue execution on same stack and update indexes
                        updateIndexes();
                        m_exitor = -1;
                    } else {
                        // Branch onto new stack with new requirements, and discard
                        // current stack
                        m_nextStack = new StateExecutionStack(m_states, nextRequirements, -1);
                        m_nextStack.schedule();

                        m_dirty = true;
                    }
                }
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (m_dirty) return;

        if (interrupted) {
            System.out.println("Cancelling states");
            for (int i = m_states.size()-1; i >= 0; i--) {
                State state = m_states.get(i);

                // Interupt active states
                state.end(InterruptType.kCancel);
                state.reset();
            }

            // Mark this stack as dirty
            m_dirty = true;
        }
    }

    @Override
    public void schedule(boolean interruptible) {
        // Check if stack was already scheduled (and discarded)
        if (m_dirty)
            throw new RuntimeException("Cannot schedule dirty stack");

        Command.super.schedule(interruptible);
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return m_requirements;
    }

    @Nullable
    public StateExecutionStack getNextStack() {
        return m_nextStack;
    }

    @Nullable
    public String[] getNextPath() {
        return m_next;
    }

    public boolean isDirty() {
        return m_dirty;
    }

    @Override
    public boolean isFinished() {
        return m_dirty == true;
    }

    private void updateIndexes() {
        System.out.println("Updated indexees");
        int size = m_states.size();
        for (int i = 0; i < size; i++) {
            m_states.get(i).setExecutionIndex(size-(i+1));
        }
    }

    public String[] getStates() {
        String[] stateNames = new String[m_states.size()];
        for (int i = 0; i < m_states.size(); i++) {
            stateNames[i] = m_states.get(i).getPath();
        }

        return stateNames;
    }
}