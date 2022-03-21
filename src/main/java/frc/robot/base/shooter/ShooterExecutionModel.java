package frc.robot.base.shooter;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.utils.Equation;
import frc.robot.utils.Range;

public class ShooterExecutionModel implements Equation, Sendable {
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

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("kA", () -> kA, null);
        builder.addDoubleProperty("kB", () -> kB, null);
        builder.addDoubleProperty("kC", () -> kC, null);
        builder.addDoubleProperty("kD", () -> kD, null);
        builder.addDoubleProperty("kOffset", () -> kOffset, null);
        builder.addStringProperty("kDomain", () -> kDomain.toString(), null);
        builder.addStringProperty("kRange", () -> kRange.toString(), null);
    }
}
