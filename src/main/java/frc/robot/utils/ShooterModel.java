package frc.robot.utils;

public class ShooterModel implements Equation {
    public final double kA;
    public final double kB;
    public final double kC;
    public final double kD;
    private final Range range;
    private final Range domain;
    
    public ShooterModel(double kA, double kB, double kC, double kD, Range domain, Range range) {
        this.kA = kA;
        this.kB = kB;
        this.kC = kC;
        this.kD = kD;
        this.domain = domain;
        this.range = range;
    }

    public double calculate(double x) {
        x = domain.normalize(x);
        return range.scale(kA*x*x*x + kB*x*x + kC*x + kD);
    }
}
