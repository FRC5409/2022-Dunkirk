package frc.robot.base.shooter;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class TrackingGains implements Sendable {
    /**
     * Feed Forward Rotation Factor
     */
    public double kFR;

    /**
     * Feed Forward Velocity Factor
     */
    public double kFV;

    /**
     * Target Compensation Factor
     */
    public double kT;

    /**
     * Feed Forward Max Output
     */
    public double kFMax;
    
    /**
     * Feed Forward Max Output
     */
    public double kFMin;

    /**
     * Target Compensation Max
     */
    public double kTMax;

    /**
     * Target Compensation Min
     */
    public double kTMin;

    public TrackingGains(
        double kFR,
        double kFV,
        double kT,
        double kFMax,
        double kFMin,
        double kTMax,
        double kTMin
    ) {
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
        builder.addDoubleProperty("kFMax", () -> kFMax, x -> kFMax = x);
        builder.addDoubleProperty("kFMin", () -> kFMin, x -> kFMin = x);
        builder.addDoubleProperty("kTMax", () -> kTMax, x -> kTMax = x);
        builder.addDoubleProperty("kTMin", () -> kTMin, x -> kTMin = x);
        builder.addDoubleProperty("kFR", () -> kFR, x -> kFR = x);
        builder.addDoubleProperty("kFV", () -> kFV, x -> kFV = x);
        builder.addDoubleProperty("kT", () -> kT, x -> kT = x);
    }
}
