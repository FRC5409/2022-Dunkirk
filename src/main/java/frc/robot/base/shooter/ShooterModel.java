package frc.robot.base.shooter;

import frc.robot.utils.Equation;
import frc.robot.utils.Range;

public class ShooterModel implements Equation {
    public final double kA;
    public final double kB;
    public final double kC;
    public final double kD;
    public final double kPitch;
    public final double kHeight;
    public final double kOffset;

    public final Range range;
    public final Range domain;
    
    public ShooterModel(
        double kA, 
        double kB, 
        double kC, 
        double kD,
        double kPitch,
        double kHeight,
        double kOffset,

        Range domain,
        Range range
    ) {
        this.kA      = kA;
        this.kB      = kB;
        this.kC      = kC;
        this.kD      = kD;
        this.kPitch  = kPitch;
        this.kHeight = kHeight;
        this.kOffset = kOffset;
        this.domain  = domain;
        this.range   = range;
    }

    public double calculate(double x) {
        x = domain.normalize(x);
        return range.scale(kA*x*x*x + kB*x*x + kC*x + kD);
    }

    public double distance(double x) {
        return kHeight / Math.tan(Math.toRadians(x + kPitch)) + kOffset;
    }
}
