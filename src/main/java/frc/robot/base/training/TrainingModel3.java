package frc.robot.base.training;

import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.utils.Equation;
import frc.robot.utils.Range;

public class TrainingModel3 implements Equation {
    public double kA;
    public double kB;
    public double kC;
    public Range  kRange;
    public Range  kDomain;
    
    public TrainingModel3(
        double kA, 
        double kB, 
        double kC,
        Range kDomain,
        Range kRange
    ) {
        this.kDomain  = kDomain;
        this.kRange   = kRange;
        this.kA       = kA;
        this.kB       = kB;
        this.kC       = kC;
    }
    
    public TrainingModel3(Range kDomain, Range kRange) {
        this(0.0, 0.0, 0.0, kDomain, kRange);
    }
    
    @Override
    public double calculate(double x) {
        x = kDomain.normalize(x);
        return kRange.scale(kA*x*x + kB*x + kC);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("kA", () -> kA, x -> kA = x);
        builder.addDoubleProperty("kB", () -> kB, x -> kB = x);
        builder.addDoubleProperty("kC", () -> kC, x -> kC = x);
        builder.addStringProperty("kDomain", () -> kDomain.toString(), null);
        builder.addStringProperty("kRange", () -> kRange.toString(), null);
    }
}
