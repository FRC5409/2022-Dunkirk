package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.MotorUtils;
import frc.robot.utils.Toggleable;

/**
 * Controls and operators the Shooter Flywheel.
 * 
 * @author Akil Pathiranage, Keith Davies
 */
public final class ShooterFlywheel extends SubsystemBase implements Toggleable {
    // Rpm to  velocity loop units
    public static final double FLYWHEEL_ROTATION_RATIO = Constants.Falcon500.unitsPerRotation / 600.0;

    // Ssensor units to rpm
    public static final double FLYWHEEL_FORWARD_RATIO = 1 / FLYWHEEL_ROTATION_RATIO;
    
    private final WPI_TalonFX           mot_main;
    private final WPI_TalonFX           mot_follower;
    private final CANSparkMax           mot_feeder;
    private final SparkMaxPIDController ctr_feeder;
    private final RelativeEncoder       enc_feeder;

    private double                      shooterTarget;
    private double                      feederTarget;

    private boolean                     enabled;

    /**
     * Constructs a flywheel subsystem.
     */
    public ShooterFlywheel() {
        mot_main = new WPI_TalonFX(Constants.kID.ShooterFalconMotor1);
            mot_main.configFactoryDefault();
            mot_main.setNeutralMode(NeutralMode.Coast);
        MotorUtils.setGains(mot_main, 0, Constants.Shooter.FLYWHEEL_GAINS);

        mot_follower = new WPI_TalonFX(Constants.kID.ShooterFalconMotor2);
            mot_follower.configFactoryDefault();
            mot_follower.setNeutralMode(NeutralMode.Coast);
            mot_follower.follow(mot_main);
            mot_follower.setInverted(TalonFXInvertType.OpposeMaster);

        mot_feeder = new CANSparkMax(Constants.kID.ShooterNeo, MotorType.kBrushless);
            mot_feeder.setSmartCurrentLimit(40);
            mot_feeder.setIdleMode(IdleMode.kBrake);
            mot_feeder.burnFlash();

        enc_feeder = mot_feeder.getEncoder();
        ctr_feeder = mot_feeder.getPIDController();
        MotorUtils.setGains(ctr_feeder, Constants.Shooter.FEEDER_GAINS);

        enabled = false;
        shooterTarget = 0;
        feederTarget = 0;
    
        if(Constants.kConfig.DEBUG){
            SmartDashboard.putNumber("Feeder Velocity", enc_feeder.getVelocity());
        }
    }
    
    /**
     * Method for enabing the flywheel.
     */
    public void enable() {
        enabled = true;
    }

    /**
     * Method for disabling the flywheel.
     */
    public void disable() {
        enabled = false;

        mot_main.stopMotor();
        mot_feeder.stopMotor();
    }

    /**
     * Method that sets the target velocity for the flywheel.
     * 
     * @param target Target revolutions per minute.
     */
    public void setVelocity(double target) {
        if (!enabled) return;

        // 600 since its rotation speed is in position change/100ms
        shooterTarget = target;
        mot_main.set(ControlMode.Velocity, shooterTarget * FLYWHEEL_ROTATION_RATIO);
    }

    /**
     * Method for spinning the feeder to an RPM.
     * 
     * @param target Target RPM.
     */
    public void spinFeeder(double target) {

        if(!enabled) return;

        feederTarget = target;
        ctr_feeder.setReference(feederTarget, CANSparkMax.ControlType.kVelocity);

    }

    /**
     * Spins the feeder at a setpoint speed from [-1.0,1.0]
     * @param setpoint
     */
    public void spinFeederSetpoint(double setpoint){
        if(!enabled) return;


        ctr_feeder.setReference(-1*setpoint, CANSparkMax.ControlType.kDutyCycle);
    }

    /**
     * Method for getting the RPM of the main motor.
     * 
     * @return A double representing the RPM of the motor.
     */
    public double getVelocity() {
        return mot_main.getSelectedSensorVelocity() * FLYWHEEL_FORWARD_RATIO;
    }

    /**
     * Method for getting if the flywheel wheels have reached the target velocity. 
     * @return true if the target has been reached, false if not.
     */
    public boolean isTargetReached() {
        return Math.abs(shooterTarget - getVelocity()) <= Constants.Shooter.FLYWHEEL_TOLERANCE;
    }

    /**
     * Method for getting if the subsystem is enabled.
     * 
     * @return True if the subsystem is enabled, false if not.
     */
    public boolean isEnabled() {
        return enabled;
    }

    /**
     * Method for stopping the feeder wheel by calling stop motor.
     */
    public void stopFeeder() {
        mot_feeder.stopMotor();
    }

    /**
     * Method for seeing if the feeder target was reached. 
     * @return returns true if it has reached the feeder target speed, false if not.
     */
    public boolean feederReachedTarget() {
        return Math.abs(feederTarget - getFeederRpm()) <= Constants.Shooter.FEEDER_TOLERANCE;
    }

    /**
     * Method for getting the feeder wheel rpm. 
     * @return The feeder RPM. 
     */
    private double getFeederRpm() {
        return enc_feeder.getVelocity();
    }

}