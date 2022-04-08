package frc.robot.base.shooter;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class TrackingGains implements Sendable {
    /**
     * Feed Forward Rotation Factor
     */
    public final double kFR;

    /**
     * Feed Forward Velocity Factor
     */
    public final double kFV;

    /**
     * Target Compensation Factor
     */
    public final double kT;

    /**
     * Feed Forward Max Output
     */
    public final double kFMax;
    
    /**
     * Feed Forward Max Output
     */
    public final double kFMin;

    /**
     * Target Compensation Max
     */
    public final double kTMax;

    /**
     * Target Compensation Min
     */
    public final double kTMin;

    /**
     * Rotation Deadband 
     */
    public final double kDeadband;

    public TrackingGains(
        double kFR,
        double kFV,
        double kT,
        double kFMax,
        double kFMin,
        double kTMax,
        double kTMin,
        double kDeadband
    ) {
        this.kDeadband = kDeadband;
        this.kFMax = kFMax;
        this.kFMin = kFMin;
        this.kTMax = kTMax;
        this.kTMin = kTMin;
        this.kFR = kFR;
        this.kFV = kFV;
        this.kT = kT;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("kDeadband", () -> kDeadband, null);
        builder.addDoubleProperty("kFMax", () -> kFMax, null);
        builder.addDoubleProperty("kFMin", () -> kFMin, null);
        builder.addDoubleProperty("kTMax", () -> kTMax, null);
        builder.addDoubleProperty("kTMin", () -> kTMin, null);
        builder.addDoubleProperty("kFR", () -> kFR, null);
        builder.addDoubleProperty("kFV", () -> kFV, null);
        builder.addDoubleProperty("kT", () -> kT, null);
    }
}
