package frc.robot.base.shooter;

import frc.robot.utils.Equation;
import frc.robot.utils.Range;

public class ShooterExecutionModel implements Equation {
    public final double kA;
    public final double kB;
    public final double kC;
    public final double kD;
    public final double kOffset;
    public final Range  kRange;
    public final Range  kDomain;
    
    public ShooterExecutionModel(
        double kA, 
        double kB, 
        double kC, 
        double kD,
        double kOffset,
        Range kDomain,
        Range kRange
    ) {
        this.kA      = kA;
        this.kB      = kB;
        this.kC      = kC;
        this.kD      = kD;
        this.kOffset = kOffset;
        this.kDomain = kDomain;
        this.kRange  = kRange;
    }

    public double calculate(double x) {
        x = kDomain.normalize(x + kOffset);
        return kRange.scale(kA*x*x*x + kB*x*x + kC*x + kD);
    }
}
