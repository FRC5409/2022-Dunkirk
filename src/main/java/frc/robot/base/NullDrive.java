package frc.robot.base;

public class NullDrive implements RobotDrive {
    @Override
    public double getEncoderVelocity() {
        return 0;
    }
}
