package frc.robot.utils;

public class PIDController {

    private double setpoint;

    private double P_GAIN;
    private double I_GAIN;
    private double D_GAIN;

    private double last_error;
    private double total_error;

    /**
     * Creates a PIDController object
     * 
     * Keeps track of error values for running pid loop
     * updates a control variable using a PID control system
     * 
     * TODO:
     *  Add an error threshold that when reached loop will
     *  return 0.
     * @param P_GAIN proportional gain
     * @param I_GAIN integral gain
     * @param D_GAIN derivative gain
     */
    public PIDController(double p, double i, double d){
        P_GAIN = p;
        I_GAIN = i;
        D_GAIN = d;

        last_error = 0;
        total_error = 0;
    }

    /**
     * Calculates the output value from the pid loop
     * 
     * @param current_state the current value of the control variable
     */
    public double calculate(double current_state){
		
		double error = setpoint - current_state;
        
        total_error += error;
		
		double out = P_GAIN * error + I_GAIN * total_error + D_GAIN * (error - last_error);
		
		last_error = error;
		
        return out;
    }

    public boolean isFinished(double setpoint){
        return Math.abs(setpoint) < 0.05;
    }

    /**
     * Updates the setpoint
     * 
     * @param newSetpoint the target values of the control variable
     */
    public void setSetpoint(double newSetpoint){
        setpoint = newSetpoint;
    }
}
