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
	
	public Gains(double _kP, double _kI, double _kD, double _kF, int _kIzone, double _kPeakOutput){
		kP = _kP;
		kI = _kI;
		kD = _kD;
		kF = _kF;
		kIzone = _kIzone;
		kPeakOutput = _kPeakOutput;
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