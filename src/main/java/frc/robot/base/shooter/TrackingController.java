package frc.robot.base.shooter;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.base.shooter.odometry.ShooterTrackingModel;
import frc.robot.utils.Vector2;

public class TrackingController {
    private final ShooterTrackingModel trackingModel;
    private final PIDController controller;

    private double kLastOutput;

    public TrackingController(ShooterTrackingModel trackingModel) {
        this.trackingModel = trackingModel;
        
        controller = new PIDController(
            trackingModel.kRotationGains.kP,
            trackingModel.kRotationGains.kI,
            trackingModel.kRotationGains.kD);

        controller.setSetpoint(0);
        controller.setTolerance(trackingModel.kTolerance);

        kLastOutput = 0;
    }

    public void update(
        Vector2 viewDirection,
        double viewOffest, 
        double turnRate, 
        double speed
    ) {
        double offset = Math.toRadians(viewOffest);
            
        Vector2 targetDirection = new Vector2(Math.cos(offset), Math.sin(offset)).unit();

        double relativeRotation = Math.atan2(
            viewDirection.x * targetDirection.y - viewDirection.y * targetDirection.x,
            viewDirection.x * targetDirection.x + viewDirection.y * targetDirection.y
        );

        kLastOutput = controller.calculate(relativeRotation) +
            turnRate * trackingModel.kRotationCompensation +
                speed * trackingModel.kVelocityCompensation;
    }

    public void reset() {
        kLastOutput = 0;
        controller.reset();
    }

    public double getOutput() {
        return kLastOutput;
    }

    public boolean isTargetReached() {
        return controller.atSetpoint();
    }
}
