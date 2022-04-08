package frc.robot.base.command;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class WaitConditionCommand extends CommandBase {
    private final Supplier<Boolean> m_condition;

    public WaitConditionCommand(Supplier<Boolean> condition) {
        m_condition = condition;
        
    }

    @Override
    public boolean isFinished() {
        return m_condition.get();
    }
}
