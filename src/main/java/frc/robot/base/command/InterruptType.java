package frc.robot.base.command;


public enum InterruptType {
    /**
     * When the state is cancelled from an external
     * interruption source.
     * 
     * <p> I.e. Calling {@link edu.wpi.first.wpilibj2.command.Command#cancel() cancel()} </p>
     */
    kCancel, 
    
    /**
     * When the state transitions to another state,
     * and has authority of the transition.
     */
    kTransition, 
    
    /**
     * When the state is detached from execution stack,
     * without authority of the transition.
     */
    kDetach,

    kFinish
}
