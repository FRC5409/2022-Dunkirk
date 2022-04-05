package frc.robot.base.training;

import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.utils.Equation;
import frc.robot.utils.Range;

public class TrainingModel4 implements Equation {
    private Setpoint kSetpoint;
    private double kA;
    private double kB;
    private double kC;
    private double kD;
    private Range kDomain;
    private Range kRange;
    
    public TrainingModel4(
        double kA, 
        double kB, 
        double kC, 
        double kD,
        Range kDomain,
        Range kRange,
        Setpoint kSetpoint
    ) {
        this.kSetpoint = kSetpoint;
        this.kDomain = kDomain;
        this.kRange  = kRange;
        this.kA      = kA;
        this.kB      = kB;
        this.kC      = kC;
        this.kD      = kD;
    }

    public TrainingModel4(
        double kA, 
        double kB, 
        double kC, 
        double kD,
        Range kDomain,
        Range kRange
    ) {
        this(kA, kB, kC, kD, kDomain, kRange, new Setpoint(kRange.mid(), kRange));
    }

    public double calculate(double x) {
        return kSetpoint.getTarget();
    }

    public double calculateReal(double x) {
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
    

    public void setModel(double kA, double kB, double kC, double kD) {
        this.kA = kA;
        this.kB = kB;
        this.kC = kC;
        this.kD = kD;
    }
    
    public void setSetpoint(Setpoint kSetpoint) {
        this.kSetpoint = kSetpoint;
    }

    public Setpoint getSetpoint() {
        return kSetpoint;
    }
}
