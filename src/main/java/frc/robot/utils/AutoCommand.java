package frc.robot.utils;

public abstract interface AutoCommand {
    public enum AutonomousState {
        kShooting, kDriving, kIntaking, kFinished
    }

    public boolean getState(AutonomousState state);
}
