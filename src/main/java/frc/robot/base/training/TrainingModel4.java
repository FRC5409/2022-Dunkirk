package frc.robot.base.training;

import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.utils.Equation;
import frc.robot.utils.Range;

public class TrainingModel4 implements Equation {
    private Setpoint kSetpoint;
    private double kDistance;
    private double kA;
    private double kB;
    private double kC;
    private double kD;
    private Range kRange;
    private Range kDomain;
    
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
        this.kDistance = 0;
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

    public TrainingModel4(Range kDomain, Range kRange) {
        this(0.0, 0.0, 0.0, 0.0, kDomain, kRange, new Setpoint(kRange.mid(), kRange));
    }
    
    @Override
    public double calculate(double x) {
        return kSetpoint.getTarget();
    }

    public double calculateReal(double x) {
        x = kDomain.normalize(x);
        return kRange.scale(kA*x*x*x + kB*x*x + kC*x + kD);
    }

    
    public void setModel(double kA, double kB, double kC, double kD) {
        this.kA = kA;
        this.kB = kB;
        this.kC = kC;
        this.kD = kD;
    }
    
    public void setDistance(double distance) {
        kDistance = distance;
    }

    public void setSetpoint(Setpoint setpoint) {
        kSetpoint = setpoint;
    }

    public Setpoint getSetpoint() {
        return kSetpoint;
    }

    public double getDistance() {
        return kDistance;
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
