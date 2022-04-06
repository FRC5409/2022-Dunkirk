package frc.robot.utils;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class Gains implements Sendable {
	public final double kP;
	public final double kI;
	public final double kD;
	public final double kF;
	public final int    kIzone;
	public final double kPeakOutput;
	
	public Gains(double kP, double kI, double kD, double kF, int kIzone, double kPeakOutput){
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		this.kF = kF;
		this.kIzone = kIzone;
		this.kPeakOutput = kPeakOutput;
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addDoubleProperty("kP", () -> kP, null);
		builder.addDoubleProperty("kI", () -> kI, null);
		builder.addDoubleProperty("kD", () -> kD, null);
		builder.addDoubleProperty("kF", () -> kF, null);
		builder.addDoubleProperty("kIzone", () -> kIzone, null);
		builder.addDoubleProperty("kPeakOutput", () -> kPeakOutput, null);
	}
}