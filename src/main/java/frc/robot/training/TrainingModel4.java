package frc.robot.training;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.utils.Equation;
import frc.robot.utils.Range;

public class TrainingModel4 implements Equation, Sendable {
    public double kA;
    public double kB;
    public double kC;
    public double kD;
    public Range  kRange;
    public Range  kDomain;
    
    public TrainingModel4(
        double kA, 
        double kB, 
        double kC, 
        double kD,
        Range kDomain,
        Range kRange
    ) {
        this.kA      = kA;
        this.kB      = kB;
        this.kC      = kC;
        this.kD      = kD;
        this.kDomain = kDomain;
        this.kRange  = kRange;
    }

    public double calculate(double x) {
        x = kDomain.normalize(x);
        return kRange.scale(kA*x*x*x + kB*x*x + kC*x + kD);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("kA", () -> kA, null);
        builder.addDoubleProperty("kB", () -> kB, null);
        builder.addDoubleProperty("kC", () -> kC, null);
        builder.addDoubleProperty("kD", () -> kD, null);
        builder.addStringProperty("kDomain", () -> kDomain.toString(), null);
        builder.addStringProperty("kRange", () -> kRange.toString(), null);
    }
}
