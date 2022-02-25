package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utils.AutoCommand;

public class AutoForward extends SequentialCommandGroup implements AutoCommand{
    private boolean m_shooting, m_driving, m_finished;

    /**
     * Creates a new shootAuto.
     */
    public AutoForward(DriveTrain sys_driveTrain) {
        addCommands(
            internal_decorate(new DriveStraight(sys_driveTrain, -0.75, -0.75), AutonomousState.kDriving).withTimeout(1)
        );
        
        m_shooting = false;
        m_driving = false;
        m_finished = false;

        /*var parent = Shuffleboard.getTab("Robot Information").getLayout("Autonomous Information", BuiltInLayouts.kList);
            parent.addBoolean("Autonomous Finished", () -> { return m_finished; });
            var child = parent.getLayout("Autonomous State", BuiltInLayouts.kGrid);
                
                child.addBoolean("Shooting State", () -> { return m_shooting; });
                child.addBoolean("Driving State", () -> { return m_driving; });*/
    }

    private Command internal_decorate(Command cmd, AutonomousState state) {
        return cmd.beforeStarting(() -> { internal_setState(state, true); })
                  .andThen(() -> { internal_setState(state, false); });
    }

    private void internal_setState(AutonomousState type, boolean state) {
        switch (type) {
            case kShooting: m_shooting = state; break;
            case kDriving: m_driving = state; break;
            case kFinished: m_finished = state; break;
            case kIntaking: break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        internal_setState(AutonomousState.kFinished, true);
        super.end(interrupted);
    }
    
    @Override
    public boolean getState(AutonomousState state) {
        switch (state) {
            case kDriving: return m_driving;
            case kShooting: return m_shooting;
            case kIntaking: return false;
            case kFinished: return m_finished;
        }
        return false;
    }
}
