package frc.robot.base.drive;

public class MockDriveOdometry implements DriveOdometry {
    @Override
    public double getEncoderVelocity() {
        return 0.0;
    }

    @Override
    public double TurnRate() {
        return 0.0;
    }
    
}
