package frc.robot.base.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.base.shooter.odometry.ShooterTrackingModel;
import frc.robot.utils.Range;
import frc.robot.utils.Vector2;

public class TrackingController {
    private final ShooterTrackingModel trackingModel;

    private double kLastOutput;
    private double kLastFeedForward;
    private double kLastReference;

    public TrackingController(ShooterTrackingModel trackingModel) {
        this.trackingModel = trackingModel;
        
        // controller = new PIDController(
        //     trackingModel.kRotationGains.kP,
        //     trackingModel.kRotationGains.kI,
        //     trackingModel.kRotationGains.kD);

        // controller.setSetpoint(0);
        // controller.setTolerance(trackingModel.kTolerance);

        // SmartDashboard.setDefaultNumber("Shooter kP", trackingModel.kRotationGains.kP);
        // SmartDashboard.setDefaultNumber("Shooter kI", trackingModel.kRotationGains.kI);
        // SmartDashboard.setDefaultNumber("Shooter kD", trackingModel.kRotationGains.kD);

        
        SmartDashboard.putData("Tracking Gains", trackingModel.kGains);

        kLastOutput = 0;
        kLastReference = 0;
        kLastFeedForward = 0;
    }

    public void update(
        Vector2 viewDirection,
        double rotation,
        double viewOffest, 
        double turnRate, 
        double speed
    ) {
        // controller.setPID(
        //     SmartDashboard.getNumber("Shooter kP", trackingModel.kRotationGains.kP),
        //     SmartDashboard.getNumber("Shooter kI", trackingModel.kRotationGains.kI),
        //     SmartDashboard.getNumber("Shooter kD", trackingModel.kRotationGains.kD)
        // );

        double offset = Math.toRadians(viewOffest);
            
        Vector2 targetDirection = new Vector2(Math.cos(offset), Math.sin(offset)).unit();

        double relativeRotation = Math.atan2(
            viewDirection.x * targetDirection.y - viewDirection.y * targetDirection.x,
            viewDirection.x * targetDirection.x + viewDirection.y * targetDirection.y
        );

        // kLastOutput = controller.calculate(relativeRotation);
        /* +
            turnRate * trackingModel.kRotationCompensation +
                speed * trackingModel.kVelocityCompensation;*/
            

        double kRotationCompensation = turnRate * trackingModel.kGains.kFR;
        double kVelocityCompensation = speed *  trackingModel.kGains.kFV;
        double kTargetCompensation = relativeRotation * trackingModel.kGains.kT;
        
        kLastFeedForward = Range.clamp(
            SmartDashboard.getNumber("Shooter FF Max", 0),
            kRotationCompensation + kVelocityCompensation,
            SmartDashboard.getNumber("Shooter FF Min", 0)
        );

        kLastReference = rotation + viewOffest + kTargetCompensation;

        SmartDashboard.putNumber("Shooter Target", kLastReference);
        SmartDashboard.putNumber("Shooter Offset", relativeRotation);
        SmartDashboard.putNumber("Shooter Offset Comp", kTargetCompensation);
        SmartDashboard.putNumber("Shooter Turn Rate", turnRate);
        SmartDashboard.putNumber("Shooter Turn Comp", kRotationCompensation);
        SmartDashboard.putNumber("Shooter Velocity", speed);
        SmartDashboard.putNumber("Shooter Velocity Comp", kVelocityCompensation);
        SmartDashboard.putNumber("Shooter Feed forward", kLastFeedForward);
    }

    public void reset() {
        kLastOutput = 0;
        kLastReference = 0;
        kLastFeedForward = 0;
        //controller.reset();
    }

    public double getOutput() {
        return kLastOutput;
    }

    public double getFeedForward() {
        return kLastFeedForward;
    }

    // public boolean isTargetReached() {
    //     return controller.atSetpoint();
    // }

    public double getReference() {
        return kLastReference;
    }
}
