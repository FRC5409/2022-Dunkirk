package frc.robot.base.shooter;

import frc.robot.base.shooter.odometry.ShooterTrackingModel;
import frc.robot.utils.Range;
import frc.robot.utils.Vector2;

public class TrackingController {
    private final ShooterTrackingModel trackingModel;

    private double kLastFeedForward;
    private double kLastReference;

    public TrackingController(ShooterTrackingModel trackingModel) {
        kLastReference = 0;
        kLastFeedForward = 0;        
        this.trackingModel = trackingModel;
    }

    public void update(
        Vector2 viewDirection,
        double rotation,
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

        double kRotationCompensation = Range.normalizeField(-180, turnRate, 180) * trackingModel.kGains.kFR;
        double kVelocityCompensation = Range.normalizeField(-1.8, speed, 1.8) *  trackingModel.kGains.kFV;
        double kTargetCompensation = relativeRotation * trackingModel.kGains.kT;

        double kNewReference = rotation - viewOffest +   
            Range.clamp(trackingModel.kGains.kTMin, kTargetCompensation, trackingModel.kGains.kTMax);
        
        if (Math.abs(kNewReference - kLastReference) > trackingModel.kGains.kDeadband)
            kLastReference = kNewReference;

        kLastFeedForward = Range.clamp(
           trackingModel.kGains.kFMin,
            kRotationCompensation + kVelocityCompensation,
            trackingModel.kGains.kFMax
        );
    }

    public void reset() {
        kLastReference = 0;
        kLastFeedForward = 0;
    }

    public double getFeedForward() {
        return kLastFeedForward;
    }

    public double getReference() {
        return kLastReference;
    }
}
